jointNumber=7

displayInfo=function(txt)
    if dlgHandle then
        sim.endDialog(dlgHandle)
    end
    dlgHandle=nil
    if txt and #txt>0 then
        dlgHandle=sim.displayDialog('info',txt,sim.dlgstyle_message,false)
        sim.switchThread()
    end
end

--getMatrixShiftedAlongZ=function(matrix,localZShift)
    -- Returns a pose or matrix shifted by localZShift along the matrix's z-axis
  --  local m={}
    --for i=1,12,1 do
      --  m[i]=matrix[i]
    --end
    --m[4]=m[4]+m[3]*localZShift
    --m[8]=m[8]+m[7]*localZShift
    --m[12]=m[12]+m[11]*localZShift
    --return m
--end

forbidThreadSwitches=function(forbid)
    -- Allows or forbids automatic thread switches.
    -- This can be important for threaded scripts. For instance,
    -- you do not want a switch to happen while you have temporarily
    -- modified the robot configuration, since you would then see
    -- that change in the scene display.
    if forbid then
        forbidLevel=forbidLevel+1
        if forbidLevel==1 then
            sim.setThreadAutomaticSwitch(false)
        end
    else
        forbidLevel=forbidLevel-1
        if forbidLevel==0 then
            sim.setThreadAutomaticSwitch(true)
        end
    end
end

findCollisionFreeConfigAndCheckApproach=function(matrix)
    -- Here we search for a robot configuration..
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 3. ..that does not collide and that can perform the IK linear approach
    sim.setObjectMatrix(ikTarget,-1,matrix)
    -- Here we check point 1 & 2:
    local c=sim.getConfigForTipPose(ikGroup,jh,0.65,1,nil,collisionPairs)
    --if not c then
      --  print('FALSE')
    --end
    if c then
        -- Here we check point 3:
        
        --local m=getMatrixShiftedAlongZ(matrix,ikShift)
        local path=generateIkPath(c,matrix,ikSteps)
        if path==nil then
            c=nil
        end
        --if path then
        --print('TRUE')
        --end
    end

    return c
end

findSeveralCollisionFreeConfigsAndCheckApproach=function(matrix,trialCnt,maxConfigs)
    -- Here we search for several robot configurations...
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 3. ..that does not collide and that can perform the IK linear approach
    forbidThreadSwitches(true)
    sim.setObjectMatrix(ikTarget,-1,matrix)
    local cc=getConfig()
    local cs={}
    local l={}
    for i=1,trialCnt,1 do
        local c=findCollisionFreeConfigAndCheckApproach(matrix)
        if c then
            local dist=getConfigConfigDistance(cc,c)
            local p=0
            local same=false
            for j=1,#l,1 do
                if math.abs(l[j]-dist)<0.001 then
                    -- we might have the exact same config. Avoid that
                    same=true
                    for k=1,#jh,1 do
                        if math.abs(cs[j][k]-c[k])>0.01 then
                            same=false
                            break
                        end
                    end
                end
                if same then
                    break
                end
            end
            if not same then
                cs[#cs+1]=c
                l[#l+1]=dist
            end
        end
        if #l>=maxConfigs then
            break
        end
    end
    forbidThreadSwitches(false)
    if #cs==0 then
        cs=nil
    end
    return cs
end

getConfig=function()
    -- Returns the current robot configuration
    local config={}
    for i=1,#jh,1 do
        config[i]=sim.getJointPosition(jh[i])
    end
    return config
end

setConfig=function(config)
    -- Applies the specified configuration to the robot
    if config then
        for i=1,#jh,1 do
            sim.setJointPosition(jh[i],config[i])
        end
    end
end

getConfigConfigDistance=function(config1,config2)
    -- Returns the distance (in configuration space) between two configurations
    local d=0
    for i=1,#jh,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end

getPathLength=function(path)
    -- Returns the length of the path in configuration space
    local d=0
    local l=#jh
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7],path[(i-1)*l+8]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7],path[i*l+8]}
        d=d+getConfigConfigDistance(config1,config2)
    end
    return d
end

followPath=function(path)
    -- Follows the specified path points. Each path point is a robot configuration. Here we don't do any interpolation
    if path then
        local l=#jh
        local pc=#path/l
        for i=1,pc,1 do
            local config={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7],path[(i-1)*l+8]}
            setConfig(config)
            sim.switchThread()
        end
    end
end

findPath=function(startConfig,goalConfigs,cnt)
    -- Here we do path planning between the specified start and goal configurations. We run the search cnt times,
    -- and return the shortest path, and its length
    local task=simOMPL.createTask('task')
    simOMPL.setAlgorithm(task,simOMPL.Algorithm.KPIECE1)
    local j1_space=simOMPL.createStateSpace('j1_space',simOMPL.StateSpaceType.joint_position,jh[1],{-180*math.pi/180},{180*math.pi/180},1)
    local j2_space=simOMPL.createStateSpace('j2_space',simOMPL.StateSpaceType.joint_position,jh[2],{-180*math.pi/180},{180*math.pi/180},2)
    local j3_space=simOMPL.createStateSpace('j3_space',simOMPL.StateSpaceType.joint_position,jh[3],{-180*math.pi/180},{180*math.pi/180},3)
    local j4_space=simOMPL.createStateSpace('j4_space',simOMPL.StateSpaceType.joint_position,jh[4],{-180*math.pi/180},{180*math.pi/180},4)
    local j5_space=simOMPL.createStateSpace('j5_space',simOMPL.StateSpaceType.joint_position,jh[5],{-180*math.pi/180},{180*math.pi/180},5)
    local j6_space=simOMPL.createStateSpace('j6_space',simOMPL.StateSpaceType.joint_position,jh[6],{-180*math.pi/180},{180*math.pi/180},6)
    simOMPL.setStateSpace(task,{j1_space,j2_space,j3_space,j4_space,j5_space,j6_space})
    
    if jointNumber==7 then
    local j7_space=simOMPL.createStateSpace('j7_space',simOMPL.StateSpaceType.joint_position,jh[7],{-180*math.pi/180},{180*math.pi/180},7)
    simOMPL.setStateSpace(task,{j1_space,j2_space,j3_space,j4_space,j5_space,j6_space,j7_space})
    elseif jointNumber==8 then
    local j7_space=simOMPL.createStateSpace('j7_space',simOMPL.StateSpaceType.joint_position,jh[7],{-180*math.pi/180},{180*math.pi/180},7)
    local j8_space=simOMPL.createStateSpace('j8_space',simOMPL.StateSpaceType.joint_position,jh[8],{-180*math.pi/180},{180*math.pi/180},8)
    simOMPL.setStateSpace(task,{j1_space,j2_space,j3_space,j4_space,j5_space,j6_space,j7_space,j8_space})
    elseif jointNumber==9 then
    local j7_space=simOMPL.createStateSpace('j7_space',simOMPL.StateSpaceType.joint_position,jh[7],{-180*math.pi/180},{180*math.pi/180},7)
    local j8_space=simOMPL.createStateSpace('j8_space',simOMPL.StateSpaceType.joint_position,jh[8],{-180*math.pi/180},{180*math.pi/180},8)
    local j9_space=simOMPL.createStateSpace('j9_space',simOMPL.StateSpaceType.joint_position,jh[9],{-180*math.pi/180},{180*math.pi/180},9)
    simOMPL.setStateSpace(task,{j1_space,j2_space,j3_space,j4_space,j5_space,j6_space,j7_space,j8_space,j9_space})
    end

    simOMPL.setCollisionPairs(task,collisionPairs)
    simOMPL.setStartState(task,startConfig)
    simOMPL.setGoalState(task,goalConfigs[1])
    for i=2,#goalConfigs,1 do
        simOMPL.addGoalState(task,goalConfigs[i])
    end
    local path=nil
    local l=999999999999
    forbidThreadSwitches(true)
    for i=1,cnt,1 do
        local res,_path=simOMPL.compute(task,4,-1,0)
        if res and _path then
            local _l=getPathLength(_path)
            if _l<l then
                l=_l
                path=_path
            end
        end
    end
    forbidThreadSwitches(false)
    simOMPL.destroyTask(task)
    return path,l
end

findShortestPath=function(startConfig,goalConfigs,searchCntPerGoalConfig)
    -- This function will search for several paths between the specified start configuration,
    -- and several of the specified goal configurations. The shortest path will be returned
    forbidThreadSwitches(true)
    local thePath=findPath(startConfig,goalConfigs,searchCntPerGoalConfig)
    forbidThreadSwitches(false)
    return thePath
end

generateIkPath=function(startConfig,goalPose,steps)
    -- Generates (if possible) a linear, collision free path between a robot config and a target pose
    forbidThreadSwitches(true)
    local currentConfig=getConfig()
    setConfig(startConfig)
    sim.setObjectMatrix(ikTarget,-1,goalPose)
    local c=sim.generateIkPath(ikGroup,jh,steps,collisionPairs)
    setConfig(currentConfig)
    forbidThreadSwitches(false)
    return c
end

getReversedPath=function(path)
    -- This function will simply reverse a path
    local retPath={}
    local ptCnt=#path/#jh
    for i=ptCnt,1,-1 do
        for j=1,#jh,1 do
            retPath[#retPath+1]=path[(i-1)*#jh+j]
        end
    end
    return retPath
end

function sysCall_threadmain()
    -- Initialization phase:
if jointNumber==9 then
    jh={-1,-1,-1,-1,-1,-1,-1,-1,-1}
elseif jointNumber==8 then
    jh={-1,-1,-1,-1,-1,-1,-1,-1}
elseif jointNumber==7 then
    jh={-1,-1,-1,-1,-1,-1,-1}
elseif jointNumber==6 then
    jh={-1,-1,-1,-1,-1,-1}
end
    for i=1,jointNumber,1 do
        jh[i]=sim.getObjectHandle('joint'..i)
    end
    ikGroup=sim.getIkGroupHandle('IK_Group')
    ikTarget=sim.getObjectHandle('target')
    collisionPairs={sim.getCollectionHandle('manipulator'),sim.getCollectionHandle('environment')}

sim.setNameSuffix(-1)

    local indicatorContainer5True=sim.addDrawingObject(sim.drawing_painttag+sim.drawing_persistent+sim.drawing_spherepoints,0.005,0,-1,9999,{0,1,0})
    local indicatorContainer4True=sim.addDrawingObject(sim.drawing_painttag+sim.drawing_persistent+sim.drawing_spherepoints,0.005,0,-1,9999,{0.75,1,0})
    local indicatorContainer3True=sim.addDrawingObject(sim.drawing_painttag+sim.drawing_persistent+sim.drawing_spherepoints,0.005,0,-1,9999,{1,1,0})
    local indicatorContainer2True=sim.addDrawingObject(sim.drawing_painttag+sim.drawing_persistent+sim.drawing_spherepoints,0.005,0,-1,9999,{1,0.5,0})
    local indicatorContainer1True=sim.addDrawingObject(sim.drawing_painttag+sim.drawing_persistent+sim.drawing_spherepoints,0.005,0,-1,9999,{1,0,0})
    local indicatorContainerFalse=sim.addDrawingObject(sim.drawing_painttag+sim.drawing_persistent+sim.drawing_spherepoints,0.005,0,-1,9999,{0,0,0})
    local indicatorContainerNaN=sim.addDrawingObject(sim.drawing_painttag+sim.drawing_persistent+sim.drawing_spherepoints,0.005,0,-1,9999,{0,0,0})
    local trueCount=0
    local falseCount=0
    local dexterityScore=0
    local fiveApproachTrue=0
    local fourApproachTrue=0
    local threeApproachTrue=0
    local twoApproachTrue=0
    local oneApproachTrue=0
    local zeroApproachTrue=0
    local basicApproachTrue=0
    local targetNumber=0
    local smallPhantom=sim.getObjectHandle('smallPhantom')
    local mediumPhantom=sim.getObjectHandle('mediumPhantom')
    local largePhantom=sim.getObjectHandle('largePhantom')
    sim.setObjectSpecialProperty(largePhantom,sim.objectspecialproperty_collidable)
    
--cascaded collidability adjustment of the different phantom layers
    modelVertexNumber=511

    for i=0,(3*modelVertexNumber)-1,1 do
    
    targetNumber=targetNumber+1
    print('Target No : '..targetNumber) 
    
    if i>(1*modelVertexNumber)-1 then
         local prop=sim.getObjectSpecialProperty(largePhantom)
         prop=sim.boolOr32(prop,sim.objectspecialproperty_collidable)-sim.objectspecialproperty_collidable
         sim.setObjectSpecialProperty(largePhantom,prop)
         sim.setObjectSpecialProperty(mediumPhantom,sim.objectspecialproperty_collidable)      

         --local prop=sim.getObjectSpecialProperty(mediumPhantom)
         --prop=sim.boolOr32(prop,sim.objectspecialproperty_collidable)-sim.objectspecialproperty_collidable
         --sim.setObjectSpecialProperty(mediumPhantom,prop)
    end

    if i>(2*modelVertexNumber)-1 then
         


         local prop=sim.getObjectSpecialProperty(mediumPhantom)
         prop=sim.boolOr32(prop,sim.objectspecialproperty_collidable)-sim.objectspecialproperty_collidable
         sim.setObjectSpecialProperty(mediumPhantom,prop)
         sim.setObjectSpecialProperty(smallPhantom,sim.objectspecialproperty_collidable)

         --local prop=sim.getObjectSpecialProperty(smallPhantom)
         --prop=sim.boolOr32(prop,sim.objectspecialproperty_collidable)-sim.objectspecialproperty_collidable
         --sim.setObjectSpecialProperty(smallPhantom,prop)
    end
   
    --if i>3 then--(3*511)-1 then
     --    sim.setObjectSpecialProperty(smallPhantom,sim.objectspecialproperty_collidable)
     --    local prop=sim.getObjectSpecialProperty(small)
       --  prop=sim.boolOr32(prop,sim.objectspecialproperty_collidable)-sim.objectspecialproperty_collidable
       --  sim.setObjectSpecialProperty(small,prop)

         --local prop=sim.getObjectSpecialProperty(smallPhantom)
         --prop=sim.boolOr32(prop,sim.objectspecialproperty_collidable)-sim.objectspecialproperty_collidable
         --sim.setObjectSpecialProperty(smallPhantom,prop)
   -- end


    target=sim.getObjectHandle('Dummy'..i)
 

   -- approachDirectionObstacle=sim.getObjectHandle('approachDirectionObstacle')
    metric={0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1}
    forbidLevel=0
    ikShift=0.1
    ikSteps=8

    --checksum to check if normal pose + 4 variations can be reached
    local checksum=0
    

        -- m is the transformation matrix or pose of the current target:
        local n=sim.getObjectMatrix(target,-1)
               indicatorPosition={n[4],n[8],n[12]}
        local nXplus30={n[1],n[2],n[3],n[4],n[5],n[6]+0.866025,n[7]-0.50000,n[8],n[9],n[10]+0.50000,n[11]+0.866025,n[12]}
        local nXminus30={n[1],n[2],n[3],n[4],n[5],n[6]+0.866025,n[7]+0.50000,n[8],n[9],n[10]-0.50000,n[11]+0.866025,n[12]}
        local nYplus30={n[1]+0.866025,n[2],n[3]+0.50000,n[4],n[5],n[6],n[7],n[8],n[9]-0.50000,n[10],n[11]+0.866025,n[12]}
        local nYminus30={n[1]+0.866025,n[2],n[3]-0.50000,n[4],n[5],n[6],n[7],n[8],n[9]+0.50000,n[10],n[11]+0.866025,n[12]}
        

        -- Compute a pose that is shifted by ikDist along the Z-axis of pose m,
        -- so that we have a final approach that is linear along target axis Z:
       -- m=getMatrixShiftedAlongZ(n,ikShift)

        -- Find several configs for pose m, and order them according to the
        -- distance to current configuration (smaller distance is better).
        -- In following function we also check for collisions and whether the
        -- final IK approach is feasable:
        --displayInfo('searching for a maximum of 60 valid goal configurations...')
        local c=findSeveralCollisionFreeConfigsAndCheckApproach(n,50,1)
        local cXplus30=findSeveralCollisionFreeConfigsAndCheckApproach(nXplus30,50,1)
        local cXminus30=findSeveralCollisionFreeConfigsAndCheckApproach(nXminus30,50,1)
        local cYplus30=findSeveralCollisionFreeConfigsAndCheckApproach(nYplus30,50,1)
        local cYminus30=findSeveralCollisionFreeConfigsAndCheckApproach(nYminus30,50,1)
        --if not c then
        --print('FALSE')
        --sim.addDrawingObjectItem(indicatorContainerFalse,indicatorPosition)
        --end
        if c then
        -- Search a path from current config to a goal config. For each goal
        -- config, search 6 times a path and keep the shortest.
        -- Do this for the first 3 configs returned by findCollisionFreeConfigs.
        -- Since we do not want an approach along the negative Z axis, we place
        -- an artificial obstacle into the scene (the blue orthogon):
        --print('TRUE')
          basicApproachTrue=basicApproachTrue+1
          checksum=checksum+1
          print('basicApproach = TRUE')
          print('Number of successful normal approaches : '..basicApproachTrue)
          
        
        
        
        
        --sim.addDrawingObjectItem(indicatorContainerTrue,indicatorPosition)
        
        --local initialApproachDirectionObstaclePose=sim.getObjectMatrix(approachDirectionObstacle,-1)
       -- sim.setObjectPosition(approachDirectionObstacle,target,{0,0,-ikShift+0.01})
        --sim.setObjectOrientation(approachDirectionObstacle,target,{0,0,0})
        --sim.switchThread() -- in order see the change before next operation locks
        --local txt='Found '..#c..' different goal configurations for the desired goal pose.'
        --txt=txt..'&&nNow searching the shortest path of 6 searches...'
        --displayInfo(txt)
        local path=findPath(getConfig(),c,6)
        --displayInfo(nil)

        --sim.setObjectMatrix(approachDirectionObstacle,-1,initialApproachDirectionObstaclePose)

        -- Follow the path:
       -- followPath(path)

        -- For the final approach, the target is the original target pose:
        --m=sim.getObjectMatrix(target,-1)
        -- Compute a straight-line path from current config to pose m:
        --path=generateIkPath(getConfig(),m,ikSteps)
        -- Follow the path:
        --followPath(path)
        -- Generate a reversed path in order to move back:
        --path=getReversedPath(path)
        -- Follow the path:
        --followPath(path)
        end
        if cXplus30 then
        checksum=checksum+1
        --local path=findPath(getConfig(),cXplus30,6)
        --followPath(path)
        end

        if cXminus30 then
        checksum=checksum+1
        --local path=findPath(getConfig(),cXminus30,6)
        --followPath(path)
        end

        if cYplus30 then
        checksum=checksum+1
        --local path=findPath(getConfig(),cYplus30,6)
        --followPath(path)
        end

        if cYminus30 then
        checksum=checksum+1
        --local path=findPath(getConfig(),cYminus30,6)
        --followPath(path)
        end
        
        if checksum==5 then
        --trueCount=trueCount+1
        print('+5')
        fiveApproachTrue=fiveApproachTrue+1
          print('Number of target points reachable in 5 orientations : '..fiveApproachTrue)
        sim.addDrawingObjectItem(indicatorContainer5True,indicatorPosition)
        dexterityScore=dexterityScore+5
        print('dexterityScore= '..dexterityScore)
        elseif checksum==4 then
        print('+4')
        fourApproachTrue=fourApproachTrue+1
          print('Number of target points reachable in 4 orientations : '..fourApproachTrue)
        sim.addDrawingObjectItem(indicatorContainer4True,indicatorPosition)
        dexterityScore=dexterityScore+4
        print('dexterityScore= '..dexterityScore)
        elseif checksum==3 then
        sim.addDrawingObjectItem(indicatorContainer3True,indicatorPosition)
        print('+3')
        threeApproachTrue=threeApproachTrue+1
          print('Number of target points reachable in 3 orientations : '..threeApproachTrue)
        dexterityScore=dexterityScore+3
        print('dexterityScore= '..dexterityScore)
        elseif checksum==2 then
        sim.addDrawingObjectItem(indicatorContainer2True,indicatorPosition)
        print('+2')
        twoApproachTrue=twoApproachTrue+1
          print('Number of target points reachable in 2 orientations : '..twoApproachTrue)
        dexterityScore=dexterityScore+2
        print('dexterityScore= '..dexterityScore)
        elseif checksum==1 then
        falseCount=falseCount+1
        sim.addDrawingObjectItem(indicatorContainer1True,indicatorPosition)
        print('+1')
        oneApproachTrue=oneApproachTrue+1
          print('Number of target points reachable in 1 orientations : '..oneApproachTrue)
        dexterityScore=dexterityScore+1
        print('dexterityScore= '..dexterityScore)
        else
        print('FALSE')
        zeroApproachTrue=zeroApproachTrue+1
          print('Number of target points reachable in 0 orientations : '..zeroApproachTrue)
        sim.addDrawingObjectItem(indicatorContainerFalse,indicatorPosition)
        end

        
    end
    print('Number of target points reachable in 0 orientations : '..zeroApproachTrue)
    print('Number of target points reachable in 1 orientations : '..oneApproachTrue)
    print('Number of target points reachable in 2 orientations : '..twoApproachTrue)
    print('Number of target points reachable in 3 orientations : '..threeApproachTrue)
    print('Number of target points reachable in 4 orientations : '..fourApproachTrue)
    print('Number of target points reachable in 5 orientations : '..fiveApproachTrue)
    print('dexterityScore= '..dexterityScore)
    
end
