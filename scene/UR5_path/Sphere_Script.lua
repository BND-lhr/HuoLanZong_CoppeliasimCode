sim = require('sim')

function sysCall_init()

    objectToFollowPath = sim.getObject('/Sphere')
    initial_path = false
    --[[
    path = sim.getObject('/Path')
    pathData = sim.unpackDoubleTable(sim.readCustomBufferData(path, 'PATH'))
    local m = Matrix(#pathData // 7, 7, pathData)
    pathPositions = m:slice(1, 1, m:rows(), 3):data()
    pathQuaternions = m:slice(1, 4, m:rows(), 7):data()
    pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)
    print('totallen = '..totalLength)
    ]]--
    velocity = 0.01 -- m/s
    posAlongPath = 0
    previousSimulationTime = 0
    sim.setStepping(true)
end

function sysCall_thread()
    if not initial_path then
        path = sim.getObject('/Path', {noError=true})
        if path ~=-1 then
            pathData = sim.unpackDoubleTable(sim.readCustomBufferData(path, 'PATH'))
            local m = Matrix(#pathData // 7, 7, pathData)
            pathPositions = m:slice(1, 1, m:rows(), 3):data()
            pathQuaternions = m:slice(1, 4, m:rows(), 7):data()
            pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)
            print('totallen = '..totalLength)
            initial_path = true
        else
            return
        end
    end
    while not sim.getSimulationStopping() do
        local t = sim.getSimulationTime()
        posAlongPath = posAlongPath + velocity * (t - previousSimulationTime)
        posAlongPath = posAlongPath % totalLength
        local pos = sim.getPathInterpolatedConfig(pathPositions, pathLengths, posAlongPath)
        --local quat = sim.getPathInterpolatedConfig(pathQuaternions, pathLengths,
                                                 --posAlongPath, nil, {2, 2, 2, 2})
        sim.setObjectPosition(objectToFollowPath, pos, path)
        --sim.setObjectQuaternion(objectToFollowPath, quat, path)
        previousSimulationTime = t
        sim.step()
    end
end

