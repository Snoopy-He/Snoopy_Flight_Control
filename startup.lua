print("hello flight control!")
local Motor={peripheral.find("Create_RotationSpeedController")}
local Motor1 = Motor[4]   --负转速        3      1
local Motor2 = Motor[3]   --正转速
local Motor3 = Motor[2]   --正转速        2      4
local Motor4 = Motor[1]   --负转速

local pi = 3.1415926

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
    Motor1.setTargetSpeed(math.floor(-denormalize(M1,0,256)))
end

function Motor2_Set()
    Motor2.setTargetSpeed(math.floor(denormalize(M2,0,256)))
end

function Motor3_Set()
    Motor3.setTargetSpeed(math.floor(-denormalize(M3,0,256)))
end

function Motor4_Set()
    Motor4.setTargetSpeed(math.floor(denormalize(M4,0,256)))
end

function Motor_Set()
    parallel.waitForAll(Motor1_Set,Motor2_Set,Motor3_Set,Motor4_Set)
end

local Yaw_Angle,Pitch_Angle,Roll_Angle = 0,0,0
local alt_pos = {
    kp = 1.5,
    ki = 0,
    kd = 0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 1000
}

local alt_spd = {
    kp = 0.05,
    ki = 0,
    kd = 0.01,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 256
}

local yaw_ang = {
    kp = 0.1,
    ki = 0,
    kd = 0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 256
}

local yaw_rat = {
    kp = 0.1,
    ki = 0.00,
    kd = 0.001,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 256
}

local pos_p = {
    kp = 0.1,
    ki = 0.0,
    kd = 0.0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 10
}

local spd = {
    kp = 0.1,
    ki = 0.001,
    kd = 0.0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 45
}

local att_ang = {
    kp = 0.7,
    ki = 0.0,
    kd = 0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 256
}

local att_rat = {
    kp = 0.02,
    ki = 0,
    kd = 0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    speed_max = 256
}

function normalize(value,min,max)
    if value < min then
        return 0
    elseif value > max then
        return 1
    else
        return value/(max-min)
    end
end

function denormalize(value,min,max)
    if value < 0  and value > -1 then
        value = (-max-min)*value
    elseif value < -1 then
        value = -max-min
    elseif value > 1 then
        value = max-min
    else 
        value = (max-min)*value
    end
    return value
end

function PID_Calc(target,current,para)
    para.error = target-current
    para.err_all = math.clamp(para.err_all + para.error,-para.errall_max,para.errall_max)
    local result = para.error * para.kp + para.err_all * para.ki + (para.error - para.last_err) * para.kd
    para.last_err = para.error
    result = math.clamp(result,-para.speed_max,para.speed_max)
    return result
end

function math.rad_reverse(rad,reverse)
    if reverse == true then
        local result
        if rad < 0 then
            result = -rad - pi
        else
            result = pi - rad
        end
        return result
    else
        return rad
    end
end

local p_re,r_re = {},{}

function Self_Check()
    if math.abs(ship.getPitch()) > 2 then
        p_re = true
    end
    if math.abs(ship.getRoll()) > 2 then
        r_re = true
    end
end

local alt,yaw,pit,rol =0,0,0,0
local pro_n = 4

function TF_P(angle,x,z)
    return -math.sin(angle) * z - math.cos(angle) * x
end

function TF_R(angle,x,z)
    return math.sin(angle) * x - math.cos(angle) * z
end

function WorldToBody(world, matrix)
    local Rinv = {
        {matrix[1][1], matrix[2][1], matrix[3][1]},
        {matrix[1][2], matrix[2][2], matrix[3][2]},
        {matrix[1][3], matrix[2][3], matrix[3][3]}
    }
    return {
        x = world.x * Rinv[1][1] + world.y * Rinv[1][2] + world.z * Rinv[1][3],
        y = world.x * Rinv[2][1] + world.y * Rinv[2][2] + world.z * Rinv[2][3],
        z = world.x * Rinv[3][1] + world.y * Rinv[3][2] + world.z * Rinv[3][3]
    }
end

function true_data_fliter(value,old_value)
    local x,y,z = value.x, value.y, value.z

    if old_value.x == nil then
        old_value.x = x
        old_value.y = y
        old_value.z = z
    end
    if math.abs(x - old_value.x) > 1 then
        x = old_value.x
    end
    if math.abs(y - old_value.y) > 1 then
        y = old_value.y
    end
    if math.abs(z - old_value.z) > 1 then
        z = old_value.z
    end

    old_value.x = x
    old_value.y = y
    old_value.z = z
    return {
        x=x,
        y=y,
        z=z
    }
end

local Pos_X = -332
local Pos_Y = 100
local Pos_Z = 77
local old_omega = {}
while true do
    Self_Check()
    local matrix = ship.getTransformationMatrix()
    local yaw_Angle = ship.getYaw()
    local pitch_Angle = math.rad_reverse(ship.getPitch(),p_re)
    local roll_Angle = math.rad_reverse(ship.getRoll(),r_re)
    local pos = ship.getWorldspacePosition()
    local omega = true_data_fliter(WorldToBody(ship.getOmega(), matrix),old_omega)
    local mass = ship.getMass()
    local vel = ship.getVelocity()
    local px = PID_Calc(Pos_X,pos.x,pos_p)
    local pz = PID_Calc(Pos_Z,pos.z,pos_p)
    local pp = TF_P(yaw_Angle,px,pz)
    local pr = TF_R(yaw_Angle,px,pz)
    local vp = TF_P(yaw_Angle,vel.x,vel.z)
    local vr = TF_R(yaw_Angle,vel.x,vel.z)
    local loit = normalize(math.sqrt(mass/1.41/pro_n/(math.cos(math.acos(math.cos(pitch_Angle)*math.cos(roll_Angle))))),0,256)    
    alt = PID_Calc(PID_Calc(Pos_Y,pos.y,alt_pos),vel.y,alt_spd) - vel.y*0.01
    --pit = PID_Calc(PID_Calc(PID_Calc(pp,vp,spd),pitch_Angle,att_ang),omega.x,att_rat)-omega.x*0.01
    --rol = PID_Calc(PID_Calc(PID_Calc(pp,vr,spd),roll_Angle,att_ang),omega.z,att_rat)-omega.z*0.01
    --yaw = PID_Calc(PID_Calc(0,yaw_Angle,yaw_ang),omega.y,yaw_rat)
    pit = PID_Calc(PID_Calc(0,pitch_Angle,att_ang),omega.x,att_rat)-omega.x*0.01  --线性阻力模拟
    rol = PID_Calc(PID_Calc(0,roll_Angle,att_ang),omega.z,att_rat)-omega.z*0.01
    M1 = alt - pit - rol - yaw + loit
    M2 = alt + pit + rol - yaw + loit
    M3 = alt - pit + rol + yaw + loit
    M4 = alt + pit - rol + yaw + loit
    --M1 = loit - 0.3 - 0.3
    --M2 = loit - 0.3 - 0.3
    --M3 = loit + 0.3 - 0.3
    --M4 = loit + 0.3 - 0.3
    --M1,M2,M3,M4 = 0,0,0,0
    print(yaw_Angle)

    Motor_Set()
end