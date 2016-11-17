-- script modificado do e-puck

-- loop principal 

r=function(num, idp)
  mult = 10^(idp or 0)
  return math.floor(num * mult + 0.5) / mult
end

threadFunction=function()
	while ( (simGetSimulationState()~=sim_simulation_advancing_abouttostop) and (state ~= 'esquerdaDireita') ) do
		st=simGetSimulationTime()
		velLeft=0
		velRight=0
		s=simGetObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
		noDetectionDistance=0.10*s
		proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
		
		for i=1,8,1 do
			res,dist=simReadProximitySensor(proxSens[i])
			if (res>0) and (dist<noDetectionDistance) then
				proxSensDist[i]=dist
			end
		end
		
		-- loop principal
						
		--if (state=='estadoInicial') then
			velRight=maxVel
			velLeft=maxVel
		--end
		
	
		-- nada à frente, parede de lado -> manter distância constante
		if (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
			
			if (math.mod(st,2)>1.8) then
				print('if inicial - estado = ' .. state)
			end
		
			if (proxSensDist[1]>0.25*noDetectionDistance) then
				if (math.mod(st,2)>1.8) then
					print('A')
				end
				--if (state=='estadoInicial') then
					velLeft=velLeft+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[1]/noDetectionDistance))
					velRight=velRight+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[1]/noDetectionDistance))
				--end
			end
			if (proxSensDist[6]>0.25*noDetectionDistance) then
				if (math.mod(st,2)>1.8) then
					print('B')
				end
				--if (state=='estadoInicial') then
					velLeft=velLeft+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[6]/noDetectionDistance))
					velRight=velRight+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[6]/noDetectionDistance))
				--end
			end
		-- alguma coisa à frente
		else
			if ( (proxSensDist[6]<noDetectionDistance) and (proxSensDist[1]<noDetectionDistance) and (proxSensDist[3]<noDetectionDistance) and (proxSensDist[4]<noDetectionDistance)) then
				print('esquerda e direita')
				print('proxSensDist[1] = ' .. proxSensDist[1])
				print('rounded = ' .. r(proxSensDist[1],2))
				print('proxSensDist[2] = ' .. proxSensDist[2])
				print('proxSensDist[3] = ' .. proxSensDist[3])
				print('proxSensDist[4] = ' .. proxSensDist[4])
				print('proxSensDist[5] = ' .. proxSensDist[5])
				print('proxSensDist[6] = ' .. proxSensDist[6])
				print('proxSensDist[7] = ' .. proxSensDist[7])
				print('proxSensDist[8] = ' .. proxSensDist[8])
				
				-- state = 'esquerdaDireita'
				
				--velLeft = 0
				--velRight = 0
			end
			
			if (math.mod(st,2)>1.8) then
					print('\nsensor 6 = ' .. proxSensDist[6] .. ', sensor 1 = ' .. proxSensDist[1] .. '\n')
			end
			
			if (math.mod(st,2)>1.8) then
					print('C')
			end
			
			
			-- Obstacle in front. Use Braitenberg to avoid it
			for i=1,4,1 do
				--if (state=='estadoInicial') then
					velLeft=velLeft+maxVel*braitFrontSens_leftMotor[i]*(1-(proxSensDist[1+i]/noDetectionDistance))
					velRight=velRight+maxVel*braitFrontSens_leftMotor[5-i]*(1-(proxSensDist[1+i]/noDetectionDistance))
				--end
			end
		end
			
		if (math.mod(st,2)>1.8) then
			print('estado fim while = ' .. state)
		end
		
		simSetJointTargetVelocity(leftMotor,velLeft)
		simSetJointTargetVelocity(rightMotor,velRight)
		simSwitchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
	end
end

-- Put some initialization code here:

state = "estadoInicial"
simSetThreadSwitchTiming(200) -- We will manually switch in the main loop
bodyElements=simGetObjectHandle('ePuck_bodyElements')
leftMotor=simGetObjectHandle('ePuck_leftJoint')
rightMotor=simGetObjectHandle('ePuck_rightJoint')
ePuck=simGetObjectHandle('ePuck')
ePuckBase=simGetObjectHandle('ePuck_base')
ledLight=simGetObjectHandle('ePuck_ledLight')
proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
for i=1,8,1 do
	proxSens[i]=simGetObjectHandle('ePuck_proxSensor'..i)
end
maxVel=120*math.pi/180
ledColors={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}

-- Braitenberg weights for the 4 front prox sensors (avoidance):
braitFrontSens_leftMotor={8,16,-2,-1}
-- Braitenberg weights for the 2 side prox sensors (following):
braitSideSens_leftMotor={-1,0}
-- Braitenberg weights for the 8 sensors (following):
braitAllSensFollow_leftMotor={-3,-1.5,-0.5,0.8,1,0,0,-4}
braitAllSensFollow_rightMotor={0,1,0.8,-0.5,-1.5,-3,-4,0}
braitAllSensAvoid_leftMotor={0,0.5,1,-1,-0.5,-0.5,0,0}
braitAllSensAvoid_rightMotor={-0.5,-0.5,-1,1,0.5,0,0,0}

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
	simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Put some clean-up code here:

