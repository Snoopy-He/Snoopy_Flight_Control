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

local M1,M2,M3,M4 = 0,0,0,0

function Motor1_Set()
    --print(math.floor(-denormalize(M1,0,2048)),-denormalize(M1,0,2048),M1)
    Motor1.setTargetSpeed(math.floor(-denormalize(M1,0,2048)))
end

function Motor2_Set()
    Motor2.setTargetSpeed(math.floor(denormalize(M2,0,2048)))
end

function Motor3_Set()
    Motor3.setTargetSpeed(math.floor(-denormalize(M3,0,2048)))
end

function Motor4_Set()
    Motor4.setTargetSpeed(math.floor(denormalize(M4,0,2048)))
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
    output_max = 1000
}

local alt_spd = {
    kp = 0.05,
    ki = 0,
    kd = 0.01,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 256
}

local yaw_ang = {
    kp = 0.1,
    ki = 0,
    kd = 0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 256
}

local yaw_rat = {
    kp = 2.0,
    ki = 0.00,
    kd = 0.2,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 256
}

local pos_p = {
    kp = 0.1,
    ki = 0.0,
    kd = 0.0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 10
}

local spd = {
    kp = 0.1,
    ki = 0.001,
    kd = 0.0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 45
}

local pit_ang = {
    kp = 0.4,
    ki = 0.0,
    kd = 0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 256
}

local pit_rat = {
    kp = 1.5,
    ki = 0,
    kd = 0.0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 1
}

local rol_ang = {
    kp = 0.4,
    ki = 0.01,
    kd = 0,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 256
}

local rol_rat = {
    kp = 1.5,
    ki = 0.0,
    kd = 0.01,
    error = 0,
    err_all = 0,
    last_err = 0,
    errall_max = 10000,
    output_max = 1
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
    if value > 1 then
        value = 1
    elseif value < -1 then
        value = -1
    end
    if value < 0 then
        value = (max - min)*(value)
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
    result = math.clamp(result,-para.output_max,para.output_max)
    return result
end

function PID_Calc_error(error,para)
    para.error = error
    para.err_all = math.clamp(para.err_all + para.error,-para.errall_max,para.errall_max)
    local result = para.error * para.kp + para.err_all * para.ki + (para.error - para.last_err) * para.kd
    para.last_err = para.error
    result = math.clamp(result,-para.output_max,para.output_max)
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

function quaternionMultiply(q1, q2)
    -- q = q1 × q2
    -- 注意：四元数乘法不满足交换律
    
    return {
        w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    }
end

function rotateQuaternionAboutAxis(q_original, axis, angle)
    -- 创建旋转四元数
    local half_angle = angle * 0.5
    local sin_half = math.sin(half_angle)
    
    local q_rotate = {
        w = math.cos(half_angle),
        x = axis.x * sin_half,
        y = axis.y * sin_half,
        z = axis.z * sin_half
    }
    
    -- 左乘：新的 = 旋转 × 原始
    return quaternionMultiply(q_rotate, q_original)

end

function axis_transfer(quat,mapping)
    if mapping == "xzy" then
        return {
            x = quat.x,
            y = quat.z,
            z = quat.y,
            w = quat.w
        }
    elseif mapping == "zxy" then
    return {
            x = quat.z,
            y = quat.x,
            z = quat.y,
            w = quat.w
        }
    else
        return quat
    end
end

function quaternionError(q_current, q_target)

    local error = {
        w = q_target.w*q_current.w + q_target.x*q_current.x + 
            q_target.y*q_current.y + q_target.z*q_current.z,
        x = q_target.w*q_current.x - q_target.x*q_current.w - 
            q_target.y*q_current.z + q_target.z*q_current.y,
        y = q_target.w*q_current.y + q_target.x*q_current.z - 
            q_target.y*q_current.w - q_target.z*q_current.x,
        z = q_target.w*q_current.z - q_target.x*q_current.y + 
            q_target.y*q_current.x - q_target.z*q_current.w
    }
    if error.w < 0 then
        error.w = -error.w
        error.x = -error.x
        error.y = -error.y
        error.z = -error.z
    end
    
    return error
end

function quaternionErrorToVector(q_error)
    -- 计算旋转角度
    local angle = 2.0 * math.acos(math.clamp(q_error.w, -1, 1))
    
    -- 计算旋转轴长度
    local axis_len = math.sqrt(q_error.x*q_error.x + q_error.y*q_error.y + q_error.z*q_error.z)
    
    if axis_len > 1e-6 then
        -- 角误差 = 角度 × 旋转轴
        local scale = angle / axis_len
        return {
            x = q_error.x * scale,
            y = q_error.y * scale,
            z = q_error.z * scale
        }
    else
        return {x = 0, y = 0, z = 0}
    end
end

function quaternionToEuler(quat)
    local w, x, y, z = quat.w, quat.x, quat.y, quat.z
    
    -- 偏航 (yaw) - 绕Z轴
    local siny_cosp = 2 * (w * z + x * y)
    local cosy_cosp = 1 - 2 * (y * y + z * z)
    local yaw = math.atan2(siny_cosp, cosy_cosp)
    
    -- 俯仰 (pitch) - 绕Y轴
    local sinp = 2 * (w * y - z * x)
    local pitch
    
    -- 处理奇异点（俯仰接近±90°）
    if math.abs(sinp) >= 1 then
        pitch = math.copysign(math.pi / 2, sinp)
    else
        pitch = math.asin(sinp)
    end
    
    -- 滚转 (roll) - 绕X轴
    local sinr_cosp = 2 * (w * x + y * z)
    local cosr_cosp = 1 - 2 * (x * x + y * y)
    local roll = math.atan2(sinr_cosp, cosr_cosp)
    return { roll = roll, pitch = pitch, yaw = yaw }
end

function angle_correct(angle,offset)
    local corrected_angle = angle + offset
    if corrected_angle > pi then
        corrected_angle = corrected_angle - 2 * pi
    elseif corrected_angle < -pi then
        corrected_angle = corrected_angle + 2 * pi
    end
    return corrected_angle
end

function euler_correct(euler,offset_r,offset_p,offset_y)
    return {
        roll = angle_correct(euler.roll, offset_r),
        pitch = -angle_correct(euler.yaw, offset_p),
        yaw = angle_correct(euler.pitch, offset_y),
    }
end

function vector(error,omega)
    return {
        error.x,
        error.y,
        error.z,
        omega.x,
        omega.y,
        omega.z
    }
end

local K = {
    {0.0010, 0.0000, -0.0000, 1.4788, 0.0000, -0.0000},
    {-0.0000, 0.0010, 0.0000, -0.0000, 1.4788, -0.0000},
    {-0.0000, 0.0000, 0.0010, -0.0000, 0.0000, 1.4788}
}

function LQR_Calc(x,k)
    local tau = {0,0,0}
    for i=1,3 do
        for j=1,6 do
            tau[i] = tau[i] - k[i][j] * x[j]
        end
    end
    return {
        x = tau[1],
        y = tau[2],
        z = tau[3]
    }
end

local Pos_X = -332
local Pos_Y = -20
local Pos_Z = 77
local old_omega = {}
local axis_offset = {x=0, y=1, z=0}  -- Y轴
local angle_offset = math.rad(45)     -- 45度转弧度

target_quat = {
    x = 0,
    y = 0,
    z = 0,
    w = 1
}
while true do
    --Self_Check()
    local matrix = ship.getTransformationMatrix()
    local yaw_Angle = ship.getYaw()
    local pitch_Angle = ship.getPitch()
    local roll_Angle = ship.getRoll()
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
    --local euler = quaternionToEuler(ship.getQuaternion())
    --local euler = quaternionErrorToVector(quaternionError(quaternionToEuler(axis_transfer(ship.getQuaternion(),"xzy")),target_quat))
    local euler = quaternionErrorToVector(quaternionError(ship.getQuaternion(), target_quat))
    local loit = normalize(math.sqrt(mass/0.15/pro_n),0,2048)    
    alt = PID_Calc(PID_Calc(Pos_Y,pos.y,alt_pos),vel.y,alt_spd) - vel.y *0.02
    --pit = PID_Calc(PID_Calc(PID_Calc(pp,vp,spd),pitch_Angle,att_ang),omega.x,att_rat)-omega.x*0.01
    --rol = PID_Calc(PID_Calc(PID_Calc(pp,vr,spd),roll_Angle,att_ang),omega.z,att_rat)-omega.z*0.01
    --yaw = PID_Calc(PID_Calc_error(euler.y,yaw_ang),omega.y,yaw_rat)
    --pit = PID_Calc(PID_Calc_error(euler.z,pit_ang),-omega.z,pit_rat)
    --rol = PID_Calc(PID_Calc_error(euler.x,rol_ang),-omega.x,rol_rat)
    --local Tau = LQR_Calc(vector(euler,omega),K)
    --M1 = alt - pit + rol + yaw + loit
    --M2 = alt + pit - rol + yaw + loit
    --M3 = alt - pit - rol - yaw + loit
    --M4 = alt + pit + rol - yaw + loit
    --M1 = alt  + (- Tau.x - Tau.z + Tau.y )/20+ loit
    --M2 = alt + (Tau.x + Tau.z + Tau.y)/20 + loit
    --M3 = alt + (- Tau.x + Tau.z - Tau.y)/20 + loit
    --M4 = alt + (Tau.x - Tau.z - Tau.y)/20 + loit

    --print(rot.x, rot.y, rot.z, rot.w)
    --M1,M2,M3,M4 = -1,-1,-1,-1
    --print(pit)
    --print(PID_Calc_error(euler.x,att_ang))


    local tensor = ship.getMomentOfInertiaTensor()
    local quat = ship.getQuaternion()
    --print(euler.roll, euler.pitch, euler.yaw)
    print(quat.w,quat.x,quat.y, quat.z)
    --print("Moment of Inertia Tensor")
    --for i=1,3,1 do
        --print(textutils.serialize(tensor[i]))
    --end


    Motor_Set()
end