
--====================================================================
local ffi = require"ffi"
--====================================================================
local ode = require "ode_ffi"
local ds  = require "drawstuff"
--====================================================================

--====================================================================
local NUM =10            -- number of boxes
local SIDE =(0.2)        -- side length of a box
local MASS =(1.0)        -- mass of a box
local RADIUS =(0.1732)   -- sphere radius
--====================================================================
local world =nil;
local space =nil;
local body  ={};
local joint ={};
local contactgroup =nil ;
local box ={};
--====================================================================
local angle = 0;

--====================================================================
function nearCallback (data, o1, o2)
    --================================================================
    local b1 = ode.dGeomGetBody(o1);
    local b2 = ode.dGeomGetBody(o2);
    --================================================================
    local MAX_CONTACTS = 8;
    local contact = ffi.new("dContact[?]",MAX_CONTACTS);
    --================================================================
    local numc = ode.dCollide ( o1
                              , o2
                              , MAX_CONTACTS
                              , contact[0].geom
                              , ffi.sizeof("dContact"));
    --================================================================
    for i=0,numc-1
    do
        contact[i].surface.mode = 0;
        contact[i].surface.mu = 1.0/0.0;
        local  c = ode.dJointCreateContact (world, contactgroup, contact+i);
        ode.dJointAttach (c, b1, b2);
    end
    --================================================================
end   
--====================================================================

--====================================================================
function nearCallBack_checkSpace(data,o1,o2)
    --================================================================
    if(    ode.dGeomIsSpace( o1 ) ~=0 
        or ode.dGeomIsSpace( o2 ) ~=0)
    then 
        local callBack = ffi.cast ( "dNearCallback",nearCallback)
        ode.dSpaceCollide2( o1, o2, data, callBack);
        --============================================================
        if( ode.dGeomIsSpace( o1 ) ~=0 )
        then
            ode.dSpaceCollide(ode.dGeomGetSpace(o1), data, callBack );
        end
        --============================================================
        if( ode.dGeomIsSpace( o2 ) ~=0 )
        then
            ode.dSpaceCollide( ode.dGeomGetSpace(o2), data, callBack );
        end 
        --============================================================
        callBack:free(); 
    else
        nearCallback (data, o1, o2);
    end 
    --================================================================
end 
--====================================================================
function simLoop (pause)
    --================================================================
    if (pause == 0) 
    then 
        
        angle =angle + 0.05;
        ode.dBodyAddForce (body[NUM-1],0,0,1.5*(-math.sin(angle)+1.0));
        --============================================================
        local callBack = ffi.cast ( "dNearCallback",nearCallBack_checkSpace)
        ode.dSpaceCollide(space,nil,callBack);
        callBack:free();
        --============================================================
        ode.dWorldQuickStep (world,0.05);
        ode.dJointGroupEmpty (contactgroup);
        --============================================================
    end
    --================================================================

    local sides =ffi.new("dReal[3]",SIDE,SIDE,SIDE);
    ds.dsSetColor (1,1,0);
    ds.dsSetTexture (ds.DS_WOOD);
    for i=0,NUM-1
    do
        ds.dsDrawBoxD(ode.dBodyGetPosition(body[i]),
                      ode.dBodyGetRotation(body[i]), sides);
    end 
    --================================================================
end
--====================================================================
function start()
    ode.dAllocateODEDataForThread(ode.dAllocateMaskAll);
    --================================================================
    local xyz = ffi.new("float[3]",2.1640,-1.3079,1.7600);
    local hpr = ffi.new("float[3]",125.5000,-17.0000,0.0000);
    ds.dsSetViewpoint (xyz,hpr);  
end
--====================================================================
function command (cmd)end 
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
    ode.dInitODE2(0);
    
    world = ode.dWorldCreate();
    space = ode.dSimpleSpaceCreate(nil);
    contactgroup = ode.dJointGroupCreate(0);
    ode.dWorldSetGravity(world, 0, 0, -0.50);
    ode.dWorldSetCFM(world,1e-5)
    
    local plane = ode.dCreatePlane(space, 0, 0, 1, 0);
    --================================================================
   
    --================================================================
    for i=0, NUM-1
    do 
        body[i] = ode.dBodyCreate (world);
        local k = i*SIDE;
        ode.dBodySetPosition  (body[i],k,k,k+0.4);
        --============================================================
        local m = ffi.new("dMass[1]");
        ode.dMassSetBox (m,1,SIDE,SIDE,SIDE);
        ode.dMassAdjust(m,MASS);
        --============================================================
        ode.dBodySetMass( body[i],m)
        --ode.dBodySetData  (  body[i] , ffi.sizeof("int") ) ;
        --============================================================
        box[i]= ode.dCreateBox (space,SIDE,SIDE,SIDE);
        ode.dGeomSetBody (box[i],body[i]);
    end 
    --================================================================
    
    --================================================================
    for i=0, NUM-1-1 
    do
        joint[i] = ode.dJointCreateBall  ( world  , nil  ) 
        ode.dJointAttach(joint[i],body[i],body[i+1]);
        local k = (i+0.5)*SIDE;
        ode.dJointSetBallAnchor (joint[i],k,k,k+0.4);
    end 
    --================================================================

    --drawstuff
    ds.dsSimulationLoop (0,nil,640, 480, func[0]);
    --================================================================
 
    --================================================================
    ode.dJointGroupDestroy(contactgroup);
    ode.dWorldDestroy(world);
    ode.dSpaceDestroy(space);
    ode.dCloseODE();
end
--====================================================================
main ()
--====================================================================
