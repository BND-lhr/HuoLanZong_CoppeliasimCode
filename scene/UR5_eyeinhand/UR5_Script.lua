sim=require'sim'
simUI=require'simUI'

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

function rad2degree(r)
    degree = r * (180/math.pi)
    return degree
end

function round(n)
    n_r = n - n%0.001
    return n_r
end

function init_file()
    -- paths require absolute path
    --[[
    -- windows
    jointFile_path = 'C:\\Git_Projects\\HuoLanZong_CoppeliasimCode\\data\\JointDegreeInfo.csv'
    poseFile_path = 'C:\\Git_Projects\\HuoLanZong_CoppeliasimCode\\data\\TipPoseInfo.csv'
    ]]--
    -- ubuntu
    jointFile_path = '/home/ss/lhr_projects/coppeliasim_test/data/JointDegreeInfo.csv'
    poseFile_path = '/home/ss/lhr_projects/coppeliasim_test/data/TipPoseInfo.csv'
    jointFile = io.open(jointFile_path, 'w')
    jointFile:close()
    poseFile = io.open(poseFile_path, 'w')
    poseFile:close()
end

function init_ui()
    xml = [[<ui title="Save info" closeable="true" placement="relative" layout="vbox" size="400,200">
        <label id="7" text="Joint1"/>
        <hslider id="1" minimum="-180" maximum="180" on-change="jointDegreeMove"/>
        <label id="8" text="Joint2"/>
        <hslider id="2" minimum="-180" maximum="180" on-change="jointDegreeMove"/>
        <label id="9" text="Joint3"/>
        <hslider id="3" minimum="-180" maximum="180" on-change="jointDegreeMove"/>
        <label id="10" text="Joint4"/>
        <hslider id="4" minimum="-180" maximum="180" on-change="jointDegreeMove"/>
        <label id="11" text="Joint5"/>
        <hslider id="5" minimum="-180" maximum="180" on-change="jointDegreeMove"/>
        <label id="12" text="Joint6"/>
        <hslider id="6" minimum="-180" maximum="180" on-change="jointDegreeMove"/>
        <button id="13" text="save joint info" on-click="addJointAngle2csv"/>
        <button id="14" text="save pose info" on-click="addPose2csv"/>
        </ui>]]
    ui = simUI.create(xml)
end

function jointDegreeMove(ui, id, v)
    local targetPos1 = {}
    for i=1, 6 do
        if i==id then degrees[i] = v end
        targetPos1[i] = degrees[i]*math.pi/180
        print(degrees[i])
    end
    print('=======================')
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,targetPos1)
    for i=1, 6 do
        simUI.setLabelText(ui, i+6, 'Joint'..i..' Degree('..degrees[i]..')')
    end
end

function addJointAngle2csv(ui, id)
    for i=1, 6 do
        print('Joint'..i..': '..degrees[i])
    end
    local file = io.open(jointFile_path, 'a')
    for i=1, 6 do
        file:write(round(degrees[i]))
        if i<6 then file:write(',') end
    end
    file:write('\n')
    file:close()
end

function addPose2csv(ui, id)
    local s = ''
    for i=1, 7 do
        --print('Pose_tip'..i..': '..round(pose[i]))
        s = s..round(pose[i])..' '
    end
    print(s)
    local file = io.open(poseFile_path, 'a')
    for i=1, 7 do
        file:write(round(pose[i]))
        if i<7 then file:write(',') end
    end
    file:write('\n')
    file:close()
end

function sysCall_init()
    degrees = {}
    jointHandles = {}
    init_file()
    init_ui()
    tip = sim.getObject('/tip')
    target = sim.getObject('/target')
    for i=1,6 do
        jointHandles[i]=sim.getObject('../joint',{index=i-1})
    end
end

function sysCall_sensing()
    pose = sim.getObjectPose(tip)
    --print('Pose_tip:'..round(pose[1]).. ', '..round(pose[2]).. ', '..round(pose[3]).. ', '..round(pose[4]).. ', '..round(pose[5]).. ', '..round(pose[6]).. ', '..round(pose[7]))
    --print('Joint Position==============================')
    for i=1,6 do
        pos = sim.getJointPosition(jointHandles[i])
        degr = round(rad2degree(pos))
        degrees[i] = degr
        --print('Joint'..i..': '..degr)
    end
end

function sysCall_thread()
    local vel=180
    local accel=40
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    --[[
    local targetPos1={90*math.pi/180,-20*math.pi/180,45*math.pi/180,-25*math.pi/180,0*math.pi/180,0*math.pi/180}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,targetPos1)
    ]]--
end
