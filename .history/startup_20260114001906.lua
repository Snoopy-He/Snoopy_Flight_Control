print("hello flight control!")
local Motor={peripheral.find("Create_RotationSpeedController")}
local Motor1 = Motor[1]   --负转速
local Motor2 = Motor[2]   --正转速
local Motor3 = Motor[4]   --正转速
local Motor4 = Motor[3]   --负转速

function Motor

function Motor_Set()
    parallel.waitForAll()
end

while true do
    local trottle = -250
    Motor1.setTargetSpeed(-trottle)
    Motor2.setTargetSpeed(trottle)
    Motor3.setTargetSpeed(trottle)
    Motor4.setTargetSpeed(-trottle)
    local omega = ship.getOmega()
    print("X: " .. omega.x)
    print("Y: " .. omega.y)
    print("Z: " .. omega.z)
    sleep(2)
end