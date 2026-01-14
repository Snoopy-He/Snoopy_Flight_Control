print("hello flight control!")
local Motor={peripheral.find("Create_RotationSpeedController")}
local Motor1 = Motor[1]   --负转速
local Motor2 = Motor[2]   --正转速
local Motor3 = Motor[4]   --正转速
local Motor4 = Motor[3]   --负转速

function Motor1_Set()
    Motor1.setTargetSpeed(-trottle)
end

function Motor2_Set()
    Motor2.setTargetSpeed(-trottle)
end

function Motor3_Set()
    Motor3.setTargetSpeed(-trottle)
end

function Motor4_Set()
    Motor4.setTargetSpeed(-trottle)
end

function Motor_Set()
    parallel.waitForAll(Motor1_Set,Motor2_Set,Motor3_Set,Motor4_Set)
end

local Yaw_Angle,Pitch_Angle,Roll_Angle
while true do
    local trottle = -250
    Motor1.setTargetSpeed(-trottle)
    Motor2.setTargetSpeed(trottle)
    Motor3.setTargetSpeed(trottle)
    Motor4.setTargetSpeed(-trottle)
    local omega = ship.getOmega()

    Yaw_Angle = Yaw_Angle + omega
    print("X: " .. omega.x)
    print("Y: " .. omega.y)
    print("Z: " .. omega.z)
end