sim=require'sim'
simUI=require'simUI'
simIK=require'simIK'

function moveToConfig(handles,maxVel,maxAccel,maxJerk,targetConf)
    local params = {
        joints = handles,
        targetPos = targetConf,
        maxVel = maxVel,
        maxAccel = maxAccel,
        maxJerk = maxJerk,
    }
    sim.moveToConfig(params)
end

function moveToPose(tiphandle,targethandle,target_pose,maxVel,maxAccel,maxJerk)
    local params = {
    ik = {
        tip = tiphandle, 
        target = targethandle
        },
    targetPose = target_pose,
    maxVel = maxVel,
    maxAccel = maxAccel,
    maxJerk = maxJerk,
    }
    sim.moveToPose(params)
end

function rad2degree(r)
    degree = r * (180/math.pi)
    return degree
end

function round(n)
    n_r = n - n%0.001
    return n_r
end

function init_JointgSpeed()
    jvel=180
    jaccel=40
    jjerk=80
    maxjVel={jvel*math.pi/180,jvel*math.pi/180,jvel*math.pi/180,jvel*math.pi/180,jvel*math.pi/180,jvel*math.pi/180}
    maxjAccel={jaccel*math.pi/180,jaccel*math.pi/180,jaccel*math.pi/180,jaccel*math.pi/180,jaccel*math.pi/180,jaccel*math.pi/180}
    maxjJerk={jjerk*math.pi/180,jjerk*math.pi/180,jjerk*math.pi/180,jjerk*math.pi/180,jjerk*math.pi/180,jjerk*math.pi/180}
    
    vel=80
    accel=1
    jerk=1
    maxVel={vel,vel,vel,vel}
    maxAccel={accel,accel,accel,accel}
    maxJerk={jerk,jerk,jerk,jerk}
end

function sysCall_init()
    degrees = {}
    jointHandles = {}
    init_JointgSpeed()
    tip = sim.getObject('/tip')
    base=sim.getObject('/UR5')
    target=sim.getObject('/target')
    for i=1,6 do
        jointHandles[i]=sim.getObject('../joint',{index=i-1})
    end
end

--[[
function sysCall_thread()
    local init_Pose={0*math.pi/180,13*math.pi/180,-38*math.pi/180,30*math.pi/180,90*math.pi/180,0*math.pi/180}
    moveToConfig(jointHandles,maxjVel,maxjAccel,maxjJerk,init_Pose)
end
]]--

function sysCall_actuation()
    local tar_pose = sim.getObjectPose(target)
    moveToPose(tip,target,tar_pose,maxVel,maxAccel,maxJerk)
end

