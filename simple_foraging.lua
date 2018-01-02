MAX_SPEED = 10

function init()
    robot.leds.set_all_colors("red")
    robot.state = "foraging"

    -- WARNING: Do not write to these two variables!
    robot.has_food = false
    robot.food_value = 0

    current_wait_steps = 0
    robot.range_and_bearing.set_data(1,0)
end

function step()

    -- Draw debug message above this robot
    robot.message = robot.id .. ": " .. robot.state .. " - " .. robot.food_value

    -- Calculate avoidance and light vectors
    local avoidance_vector = getAvoidanceVector()
    local light_vector = getLightVector()

    -- Foraging finite state machine
    if robot.state == "foraging" then
        foraging()
    elseif robot.state == "returning" then
        returning()
    elseif robot.state == "waiting" then
        waiting()
    end

end

function reset()
    init()
end

function destroy()

end

function waiting()
    if current_wait_steps > 0 then
        --while waiting, broadcast food value
        current_wait_steps = current_wait_steps - 1
        robot.range_and_bearing.set_data(1, robot.food_value)
    --finished waiting, so change state and stop broadcasting
    else
        robot.range_and_bearing.set_data(1, 0)
        current_wait_steps = 3 * robot.food_value
        robot.range_and_bearing.set_data(2, current_wait_steps)
        robot.leds.set_all_colors("green")
        robot.state = "returning"
    end
end

function foraging()

    -- If the robot has picked up a food item
    if robot.has_food then
        robot.wheels.set_velocity(0, 0)
        robot.state = "waiting"
        robot.leds.set_all_colors("green")
        --robot wait steps is lower for lower food values
        current_wait_steps = 2 * robot.food_value

    -- If the robot doesn't have a food item
    else
        -- If the robot is in the nest
        if inNest() then
            -- Perform anti-phototaxis, while avoiding obstacles
            setWheelSpeedsFromVector(subtractVectors(getAvoidanceVector(), getLightVector()))
        -- If the robot is outside the nest
        else
            --avoid obstacles while heading towards the food source
            setWheelSpeedsFromVector(addVectors(getAvoidanceVector(), getFoodSourceVector()))
        end
    end

end

function returning()

    -- If the robot has deposited its food item at the nest
    if not robot.has_food then
        -- Set LEDs to red, and transition to foraging state
        robot.leds.set_all_colors("red")
        robot.state = "foraging"
        --reset range and bearing data
        robot.range_and_bearing.set_data(2, 0)
    -- If the robot has a food item
    else
        --robot broadcasts that it has food, with value decaying to nothing over time
        --thus, robots closer to the food source will broadcast a higher value.
        --foraging bots can move towards the highest value detected to get closer to the food source
        if current_wait_steps > 0 then
            current_wait_steps = current_wait_steps - 1
            robot.range_and_bearing.set_data(2, current_wait_steps)
        end
        -- Perform phototaxis, while avoiding obstacles
        setWheelSpeedsFromVector(addVectors(getAvoidanceVector(), getLightVector()))
        
    end

end

function getAvoidanceVector()

    local avoidance_vector = { x = 0, y = 0 }

    -- Calculate vector to obstacles (if any)
    for i = 1, #robot.proximity do
        avoidance_vector = addVectors(avoidance_vector, newVectorFromPolarCoordinates(robot.proximity[i].value, robot.proximity[i].angle));
    end

    -- If there are no obstacles straight ahead
    if (getVectorAngleDegrees(avoidance_vector) > -5 and getVectorAngleDegrees(avoidance_vector) < 5) and getVectorLength(avoidance_vector) < 0.1 then
        -- Return a unit vector along the x-axis (straight ahead)
        return newVectorFromPolarCoordinates(1, 0)
    else
        -- Otherwise, return a unit vector 180 degrees away from the obstacle
        avoidance_vector = normaliseVectorLength(avoidance_vector)
        return { x = -avoidance_vector.x, y = -avoidance_vector.y }
    end

end

function getLightVector()

    local light_vector = { x = 0, y = 0 }

    -- Calculate vector to lights
    for i = 1, #robot.light do
        light_vector = addVectors(light_vector, newVectorFromPolarCoordinates(robot.light[i].value, robot.light[i].angle));
    end

    -- If any light was detected
    if getVectorLength(light_vector) > 0 then
        -- Return a unit vector towards the light source
        return newVectorFromPolarCoordinates(1, getVectorAngleRadians(light_vector))
    else
        -- Otherwise, return a zero vector
        return { x = 0, y = 0 }
    end

end

function getFoodSourceVector()
    local food_source_vector = { x = 0, y = 0 }
    local temp = 0

    for i = 1, #robot.range_and_bearing do
        --if nest 15 is detected, set heading for that and break
        if robot.range_and_bearing[i].data[1] == 15 then
            food_source_vector = newVectorFromPolarCoordinates(robot.range_and_bearing[i].range, robot.range_and_bearing[i].horizontal_bearing)
            count = -1
            break
        --else turn towards the highest broadcasted food value in the second range and bearing location
        --this value decays as the robots travel back towards the nest, so robots without food should not
        --follow robots with food back to the nest    
        elseif robot.range_and_bearing[i].data[2] > 0 then
            if robot.range_and_bearing[i].data[2] > temp then
                food_source_vector = newVectorFromPolarCoordinates(robot.range_and_bearing[i].range, robot.range_and_bearing[i].horizontal_bearing)
                temp = robot.range_and_bearing[i].data[2]
            end
        end
    end

    return food_source_vector
end

function setWheelSpeedsFromVector(vector)

    robot.vectors = { { x = normaliseVectorLength(vector).x, y = normaliseVectorLength(vector).y, color = "yellow" } }

    -- Normalise angle of vector
    local heading_angle = normaliseAngle(getVectorAngleDegrees(vector))

    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    -- Turn left or right, based on angle of vector
    if heading_angle > 10 then
        left_speed = 0
    elseif heading_angle < -10 then
        right_speed = 0
    end

    -- Set wheel speeds
    robot.wheels.set_velocity(left_speed, right_speed)

end

function inNest()

    -- If the two rear motor ground sensors detect grey, then the robot is completely within the nest
    if robot.motor_ground[2].value > 0.25 and
       robot.motor_ground[2].value < 0.75 and
       robot.motor_ground[3].value > 0.25 and
       robot.motor_ground[3].value < 0.75 then

        return true
    else
        return false
    end

end

-- Utility functions
function newVectorFromPolarCoordinates(length, angle_radians)
    return { x = (length * math.cos(angle_radians)),
             y = (length * math.sin(angle_radians)) }
end

function addVectors(a, b)
    return { x = a.x + b.x,
             y = a.y + b.y }
end

function subtractVectors(a, b)
    return { x = a.x - b.x,
             y = a.y - b.y }
end

function getVectorAngleRadians(vector)
    return math.atan2(vector.y, vector.x)
end

function getVectorAngleDegrees(vector)
    return math.deg(getVectorAngleRadians(vector))
end

function getVectorLength(vector)
    return math.sqrt((vector.x * vector.x) + (vector.y * vector.y))
end

function normaliseVectorLength(vector)
    local length = getVectorLength(vector)
    return { x = vector.x / length,
             y = vector.y / length}
end

function normaliseAngle(angle_degrees)
    while angle_degrees > 180 do
        angle_degrees = angle_degrees - 360
    end

    while angle_degrees < -180 do
        angle_degrees = angle_degrees + 360
    end

    return angle_degrees
end