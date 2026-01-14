print("hello flight control!")
local Motor={peripheral.find("Create_RotationSpeedController")}
local Motor1 = Motor[1]   --负转速        3      1
local Motor2 = Motor[2]   --正转速
local Motor3 = Motor[4]   --正转速        2      4
local Motor4 = Motor[3]   --负转速

function math.clamp(value,min,max)
    if value<min then
        return min
    end
    if value>max then
        return max
    end
    return value
end

local trottle = -100
local M1,M2,M3,M4 = 0,0,0,0

function Motor1_Set()
    Motor1.setTargetSpeed(-M1)
end

function Motor2_Set()
    Motor2.setTargetSpeed(M2)
end

function Motor3_Set()
    Motor3.setTargetSpeed(M3)
end

function Motor4_Set()
    Motor4.setTargetSpeed(-M4)
end

function Motor_Set()
    parallel.waitForAll(Motor1_Set,Motor2_Set,Motor3_Set,Motor4_Set)
end

local Yaw_Angle,Pitch_Angle,Roll_Angle = 0,0,0
local alt_para = {
    kp = 8,
    ki = 0.03,
    kd = 0.05,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 256
}

local yaw_para = {
    kp = 0.05,
    ki = 0,
    kd = 0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 0,
    speed_max = 256
}

local pit_para = {
    kp = 5,
    ki = 0.01,
    kd = 0.1,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 0,
    speed_max = 256
}

local rol_para = {
    kp = 5,
    ki = 0.01,
    kd = 0.1,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 0,
    speed_max = 256
}

function PID_Calc(current,target,para)
    para.error = target-current
    --print("err"..para.error)
    para.err_all = math.clamp(para.err_all + para.error,-para.errall_max,para.errall_max)
    local result = para.error * para.kp + para.err_all * para.ki + (para.error - para.last_err) * para.kd
    para.last_err = para.error
    result = math.clamp(result,-para.speed_max,para.speed_max)
    return result
end

local alt,yaw,pit,rol
while true do
    local yaw_Angle = ship.getYaw()
    local pitch_Angle = ship.getPitch()
    local roll_Angle = ship.getRoll()
    local pos = ship.getWorldspacePosition()
    alt = PID_Calc(pos.y,0,alt_para)
    yaw = PID_Calc(yaw_Angle,0,yaw_para)
    pit = PID_Calc(pitch_Angle,0,pit_para)
    rol = PID_Calc(roll_Angle,0,rol_para)
    print("alt:"..pos.y)
    M1 = alt + pit - rol
    M2 = alt - pit + rol
    M3 = alt - pit - rol
    M4 = alt + pit + rol

    Motor_Set()
end