print("hello flight control!")
local Motor={peripheral.find("Create_RotationSpeedController")}
local Motor1 = Motor[1]   --负转速
local Motor2 = Motor[2]   --正转速
local Motor3 = Motor[4]   --正转速
local Motor4 = Motor[3]   --负转速

while true do
    Motor1.setTargetSpeed(-0)
    Motor2.setTargetSpeed(0)
    Motor3.setTargetSpeed(0)
    Motor4.setTargetSpeed(-0)
    local q = quaternion.new()
local pitch, yaw, roll = q:toEuler()
    sleep(2)
end