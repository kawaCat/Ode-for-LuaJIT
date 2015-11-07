
--====================================================================
local ffi =require"ffi"
--====================================================================
local ode = require "ode_ffi"
local ds =require "drawstuff"
--====================================================================
-- for ode state save
--====================================================================
ffi.cdef(
[[
    FILE * fopen ( const char * filename, const char * mode );
    int fclose ( FILE * stream );    
]])
--====================================================================

--====================================================================
local LENGTH =0.7    -- chassis length
local WIDTH  =0.5    -- chassis width
local HEIGHT =0.2    -- chassis height
local RADIUS =0.18   -- wheel radius
local STARTZ =0.5    -- starting height of chassis
local CMASS  =1      -- chassis mass
local WMASS  =0.2    -- wheel mass
--====================================================================
-- dynamics and collision objects (chassis, 3 wheels, environment)
local world = nil;-- world
local space  =nil --space;
local car_space=nil;
local body={};
local joint={};    -- joint[0] is the front wheel
local contactgroup =nil;
local ground=nil;
local box={};
local sphere={};
local ground_box=nil;
--====================================================================
local speed=0
local steer=0;    -- user commands

--====================================================================
function nearCallback (data, o1, o2)
    -- only collide things with the ground
    local g1 = (o1 == ground ) or ( o1 == ground_box) or false;
    local g2 = (o2 == ground ) or ( o2 == ground_box) or false;
    --================================================================
    -- skip ground
    if (   g1 == true  and g2 == true 
        or g1 == false and g2 == false )
    then 
        return; 
    end ;
    --================================================================
    local N = 10;
    local contact = ffi.new("dContact[?]",N);
    --================================================================
    local n = ode.dCollide (o1,o2,N,contact[0].geom,ffi.sizeof("dContact"));
    --================================================================
    if (n > 0)
    then 
        for i=0, n-1
        do
          contact[i].surface.mode =   ode.dContactSlip1     
                                    + ode.dContactSlip2 
                                    + ode.dContactSoftERP 
                                    + ode.dContactSoftCFM 
                                    + ode.dContactApprox1;
          contact[i].surface.mu = (1.0/0.0) -- inf(dInfinity)
          contact[i].surface.slip1 = 0.1;
          contact[i].surface.slip2 = 0.1;
          contact[i].surface.soft_erp = 0.5;
          contact[i].surface.soft_cfm = 0.3;
          local c = ode.dJointCreateContact (world,contactgroup,contact);
          ode.dJointAttach ( c
                           , ode.dGeomGetBody(contact[i].geom.g1)
                           , ode.dGeomGetBody(contact[i].geom.g2));
        end
    end
end   
--====================================================================

--====================================================================
-- determines whether a space
-- from.
-- http://so-zou.jp/robot/tech/physics-engine/ode/collision-detection/sample-code.htm
--====================================================================
function nearCallBack_checkSpace(data,o1,o2)
    -- google translate.
    -- o1, one of o2 determines whether a space .
    if(    ode.dGeomIsSpace( o1 ) ~=0 
        or ode.dGeomIsSpace( o2 ) ~=0)
    then 
        -- o1, o2 also in consideration is a different space , and
        -- performs collision detection.
        local callBack = ffi.cast ( "dNearCallback",nearCallback)
        ode.dSpaceCollide2( o1, o2, data, callBack);
        
        --============================================================
        if( ode.dGeomIsSpace( o1 ) ~=0 )
        then
            -- if o1 is space , and
            -- the collision detection to target all of the geometry contained therein
            ode.dSpaceCollide(ode.dGeomGetSpace(o1), data, callBack );
        end
        --============================================================
        if( ode.dGeomIsSpace( o2 ) ~=0 )
        then
            -- if o2 is space , and
            -- the collision detection to target all of the geometry contained therein
            ode.dSpaceCollide( ode.dGeomGetSpace(o2), data, callBack );
        end 
        --============================================================
        callBack:free(); -- free luajit ffi callback.
    else
        nearCallback (data, o1, o2);
    end 
end 
--====================================================================

-- drawstuff simlation loop func
--====================================================================
function simLoop (pause)
    
    if (pause == 0 ) -- pause==1 : pause
    then 
        -- motor
        ode.dJointSetHinge2Param (joint[0],ode.dParamVel2,-speed);
        ode.dJointSetHinge2Param (joint[0],ode.dParamFMax2,0.1);
        -- steering
        local v = steer - ode.dJointGetHinge2Angle1 (joint[0]);
        --============================================================
        if (v > 0.1) then  v = 0.1;  end
        if (v < -0.1)then  v = -0.1; end 
        --============================================================
        v = v * 10.0;
        --============================================================
        ode.dJointSetHinge2Param (joint[0],ode.dParamVel1,v);
        ode.dJointSetHinge2Param (joint[0],ode.dParamFMax,0.2);
        ode.dJointSetHinge2Param (joint[0],ode.dParamLoStop,-0.75);
        ode.dJointSetHinge2Param (joint[0],ode.dParamHiStop,0.75);
        ode.dJointSetHinge2Param (joint[0],ode.dParamFudgeFactor,0.1);
        --============================================================
        local callBack = ffi.cast ( "dNearCallback",nearCallBack_checkSpace)
        ode.dSpaceCollide(space,nil,callBack);
        callBack:free();
        --============================================================
        ode.dWorldStep (world,0.05);
        
        -- remove all contact joints
        ode.dJointGroupEmpty (contactgroup);
        --============================================================
    end
    --================================================================
    ds.dsSetColor (0,1,1);
    ds.dsSetTexture (ds.DS_WOOD);
    local sides =ffi.new("dReal[3]",LENGTH,WIDTH,HEIGHT);
    ds.dsDrawBoxD (ode.dBodyGetPosition(body[0]),ode.dBodyGetRotation(body[0]),sides);
    ds.dsSetColor (1,1,1);
    --================================================================
    for i=1, 3 
    do
        ds.dsDrawCylinderD (ode.dBodyGetPosition(body[i]),
                        ode.dBodyGetRotation(body[i]),0.02,RADIUS);
    end 
    --================================================================
    local ss = ffi.new("dVector3[1]");
    ode.dGeomBoxGetLengths (ground_box,ss[0]);
    ds.dsDrawBoxD (ode.dGeomGetPosition(ground_box),ode.dGeomGetRotation(ground_box),ss[0]);
    --================================================================
end
--====================================================================

-- drawstuff start func
--====================================================================
function start()
    
    ode.dAllocateODEDataForThread(ode.dAllocateMaskAll);
   
    -- initial camera position
    local  xyz = ffi.new("float[3]",0.8317, -0.9817, 0.8000);
    local  hpr = ffi.new("float[3]",121.0000, -27.5000, 0.0000);
    ds.dsSetViewpoint (xyz,hpr);
    
    print ([[
    Press:
     'a' to increase speed.
     'z' to decrease speed.
     ',' to steer left.
     '.' to steer right.
     ' ' to reset speed and steering.
     '1' to save the current state to 'state.dif'.]]);
end
--====================================================================
-- called when a key pressed
function command (cmd)
    if (   cmd == string.byte'a' 
        or cmd == string.byte'A' )
    then 
        speed =speed+ 0.3;
    elseif (   cmd == string.byte'z' 
            or cmd == string.byte'Z' )
    then 
        speed =speed- 0.3;
    elseif (cmd == string.byte',')
    then 
        steer = steer -0.5;
        if ( steer <-1)
        then 
            steer = -1;
        end 
    elseif (cmd == string.byte'.'  )
    then 
        steer = steer +0.5;
        if ( steer > 1)
        then 
            steer = 1;
        end 
    elseif (cmd == string.byte' '  )
    then 
        speed = 0;
        steer = 0;
    elseif (cmd == string.byte'1' )
    then 
        local file  = ffi.C.fopen ("state.dif","wt");
        if ( file ~=nil )
        then 
            ode.dWorldExportDIF (world,f,"");
            ffi.C.fclose (file);
        end 
        print("save file as state.dif");
    end
end 
--====================================================================

--====================================================================
function main ()
    --================================================================
    --drawstuff function struct
    local func = ffi.new("dsFunctions[1]");
    func[0].version = ds.DS_VERSION;
    func[0].start = start;
    func[0].step = simLoop;
    func[0].command = command;     
    func[0].stop  = nil;       
    func[0].path_to_textures = "./textures";
    --================================================================
    -- create world
    ode.dInitODE2(0);
    world = ode.dWorldCreate();
    
    space = ode.dHashSpaceCreate (nil);
    contactgroup = ode.dJointGroupCreate (0);
    ode.dWorldSetGravity (world,0,0,-0.5);
    ground = ode.dCreatePlane (space,0,0,1,0);
    --================================================================
    
    --================================================================
    local m = ffi.new("dMass[1]");
    -- chassis body
    body[0] = ode.dBodyCreate (world);
    ode.dBodySetPosition (body[0],0,0,STARTZ);
    ode.dMassSetBox (m,1,LENGTH,WIDTH,HEIGHT);
    ode.dMassAdjust (m,CMASS);
    ode.dBodySetMass (body[0],m);
    box[0] =ode.dCreateBox (nil,LENGTH,WIDTH,HEIGHT);
    ode.dGeomSetBody (box[0],body[0]);
    
    -- wheel bodies
    for i=1,3
    do
        body[i] = ode.dBodyCreate (world);
        local q = ffi.new("dQuaternion[1]");
        ode.dQFromAxisAndAngle (q[0],1,0,0,math.pi*0.5);
        ode.dBodySetQuaternion (body[i],q[0]);
        ode.dMassSetSphere (m,1,RADIUS);
        ode.dMassAdjust (m,WMASS);
        ode.dBodySetMass (body[i],m);
        sphere[i-1] = ode.dCreateSphere (nil,RADIUS);
        ode.dGeomSetBody (sphere[i-1],body[i]);
    end
    --================================================================
    ode.dBodySetPosition (body[1], 0.5*LENGTH,0,STARTZ-HEIGHT*0.5);
    ode.dBodySetPosition (body[2],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
    ode.dBodySetPosition (body[3],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);
    --================================================================
        
    -- front and back wheel hinges
    for i=0, 3-1
    do
        joint[i] = ode.dJointCreateHinge2 (world,nil);
        ode.dJointAttach (joint[i],body[0],body[i+1]);
        local  a = ode.dBodyGetPosition (body[i+1]);
        ode.dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
        ode.dJointSetHinge2Axis1 (joint[i],0,0,1);
        ode.dJointSetHinge2Axis2 (joint[i],0,1,0);
    end

    -- set joint suspension
    for i=0, 3-1
    do
        ode.dJointSetHinge2Param (joint[i],ode.dParamSuspensionERP,0.4);
        ode.dJointSetHinge2Param (joint[i],ode.dParamSuspensionCFM,0.8);
    end
    --================================================================

    -- lock back wheels along the steering axis
    for i=1,3-1
    do
        -- set stops to make sure wheels always stay in alignment
        ode.dJointSetHinge2Param (joint[i],ode.dParamLoStop,0);
        ode.dJointSetHinge2Param (joint[i],ode.dParamHiStop,0);
    end 
    --================================================================
    
    -- create Car Space 
    car_space = ode.dSimpleSpaceCreate (space);
    ode.dSpaceSetCleanup (car_space,0);
    ode.dSpaceAdd (car_space,box[0]);
    ode.dSpaceAdd (car_space,sphere[0]);
    ode.dSpaceAdd (car_space,sphere[1]);
    ode.dSpaceAdd (car_space,sphere[2]);
    
    -- environment
    ground_box = ode.dCreateBox (space,2,1.5,1);
    local  R = ffi.new("dMatrix3[1]");
    ode.dRFromAxisAndAngle (R[0],0,1,0,-0.15);
    ode.dGeomSetPosition (ground_box,2,0,-0.34);
    ode.dGeomSetRotation (ground_box,R[0]);
    --================================================================
    
    --drawstuff
    ds.dsSimulationLoop (0,nil,352,288,func[0]);
    --================================================================
    
    ode.dGeomDestroy (box[0]);
    ode.dGeomDestroy (sphere[0]);
    ode.dGeomDestroy (sphere[1]);
    ode.dGeomDestroy (sphere[2]);
    ode.dJointGroupDestroy (contactgroup);
    ode.dSpaceDestroy (space);
    ode.dWorldDestroy (world);
    ode.dCloseODE();
end
--====================================================================
main ()
--====================================================================
