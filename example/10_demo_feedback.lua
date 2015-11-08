--====================================================================
local ffi = require"ffi"
--====================================================================
local ode = require "ode_ffi"
local ds  = require "drawstuff"
--====================================================================

--====================================================================
local world = nil;
local space = nil;
local contactgroup = nil ;
--====================================================================
local STACKCNT = 10;    -- nr of weights on bridge
local SEGMCNT  = 16;    -- nr of segments in bridge
local SEGMDIM  = ffi.new("dReal[3]", 0.9, 4, 0.1 );

local groundgeom;
local segbodies={};
local seggeoms={};
local stackbodies={};
local stackgeoms={};
local hinges={};
local sliders={};

local colours={};
local stress={};
local jfeedbacks=ffi.new("dJointFeedback[?]",SEGMCNT-1);

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
    if (numc > 0)
    then 
        for i=0, numc-1
        do
            contact[i].surface.mode =  ode.dContactSoftERP 
                                     + ode.dContactSoftCFM 
                                     + ode.dContactApprox1;
            contact[i].surface.mu = 100.0;
            contact[i].surface.soft_erp = 0.96;
            contact[i].surface.soft_cfm = 0.02;
            
            local c = ode.dJointCreateContact (world,contactgroup,contact);
            ode.dJointAttach ( c
                           , ode.dGeomGetBody(contact[i].geom.g1)
                           , ode.dGeomGetBody(contact[i].geom.g2));
        end
    end
    --================================================================
end   
--====================================================================
function nearCallBack_checkSpace(data,o1,o2)
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
end 
--====================================================================

--====================================================================
function drawGeom ( g)
    local pos = ode.dGeomGetPosition(g);
    local R   = ode.dGeomGetRotation(g);
    local type_ = ode.dGeomGetClass (g);
    --================================================================
    if (type_ == ode.dBoxClass)
    then 
        local sides=ffi.new("dVector3[1]");
        ode.dGeomBoxGetLengths (g, sides[0]);
        ds.dsDrawBoxD (pos,R,sides[0]);
    end
    --================================================================
    if (type_ == ode.dCylinderClass)
    then
        local r =ffi.new("dReal[1]");
        local l=ffi.new("dReal[1]");
        ode.dGeomCylinderGetParams(g, r, l);
        ds.dsDrawCylinderD(pos, R, l[0], r[0]);
    end
end
--====================================================================

-- from odemath.h. (has not export func)
function dCalcVectorLength3(a)
    return math.sqrt((a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]));
end
--====================================================================

--====================================================================
function inspectJoints()
    local forcelimit = 2000.0;
    --================================================================
    for i=0, SEGMCNT-1-1
    do
        if ( ode.dJointGetBody(hinges[i], 0) ~=nil)
        then
            -- This joint has not snapped already... inspect it.
            local l0 = dCalcVectorLength3(jfeedbacks[i].f1);
            local l1 = dCalcVectorLength3(jfeedbacks[i].f2);
            colours[i+0] = 0.95*colours[i+0] + 0.05 * l0/forcelimit;
            colours[i+1] = 0.95*colours[i+1] + 0.05 * l1/forcelimit;
            --========================================================
            if (   l0 > forcelimit 
                or l1 > forcelimit )
            then 
                stress[i] = stress[i] + 1;
            else
                stress[i] = 0;
            end 
            --========================================================
            if ( stress[i] > 4)
            then 
                ode.dJointAttach (hinges[i], nil,nil);
            end 
        end
    end
end
--====================================================================

--====================================================================
function simLoop (pause)
    --================================================================
    local simstep = 0.002; -- 2ms simulation steps
    local dt = ds.dsElapsedTime();
    local nrofsteps =math.floor((dt/simstep)+0.5);
    --================================================================
    if (pause == 0 ) -- pause==1 : pause
    then 
        for i=0, nrofsteps-1 
        do
            local callBack = ffi.cast ( "dNearCallback",nearCallback)
            ode.dSpaceCollide(space,nil,callBack);
            callBack:free();
            --========================================================
            ode.dWorldQuickStep (world,simstep);
            ode.dJointGroupEmpty (contactgroup);
            --========================================================
            inspectJoints();
        end
    end
    --================================================================
    for i=0, SEGMCNT-1
    do
        local r=0; local g=0; local b=0.2;
        local v = colours[i];
        --============================================================
        if (v>1.0) then v=1.0;end 
        if (v<0.5) 
        then
            r=2*v;
            g=1.0;
        else
            r=1.0;
            g=2*(1.0-v);
        end 
        --============================================================
        ds.dsSetColor (r,g,b);
        drawGeom(seggeoms[i]);
    end
    --================================================================
    ds.dsSetColor (1,1,1);
    --================================================================
    for i=0,STACKCNT-1
    do
        drawGeom(stackgeoms[i]);
    end 
end
--====================================================================
function start()
    ode.dAllocateODEDataForThread(ode.dAllocateMaskAll);
    --================================================================
    local xyz = ffi.new("float[3]",-6, 8, 6);
    local hpr = ffi.new("float[3]",-65.0, -27.0, 0.0);
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
    space = ode.dHashSpaceCreate (nil);
    contactgroup = ode.dJointGroupCreate (0);
    ode.dWorldSetGravity (world,0,0,-9.8);
    ode.dWorldSetQuickStepNumIterations (world, 20);
    
    --================================================================
    local m = ffi.new("dMass[1]");
    --================================================================
    for i=0,SEGMCNT-1
    do
        segbodies[i] = ode.dBodyCreate (world);
        ode.dBodySetPosition(segbodies[i], i - SEGMCNT/2.0, 0, 5);
        ode.dMassSetBox (m, 1, SEGMDIM[0], SEGMDIM[1], SEGMDIM[2]);
        ode.dBodySetMass (segbodies[i], m);
        seggeoms[i] = ode.dCreateBox (nil, SEGMDIM[0], SEGMDIM[1], SEGMDIM[2]);
        ode.dGeomSetBody (seggeoms[i], segbodies[i]);
        ode.dSpaceAdd (space, seggeoms[i]);
    end
    --================================================================
    for i=0,SEGMCNT-1-1
    do
        hinges[i] = ode.dJointCreateHinge (world,nil);
        ode.dJointAttach (hinges[i], segbodies[i],segbodies[i+1]);
        ode.dJointSetHingeAnchor (hinges[i], i + 0.5 - SEGMCNT/2.0, 0, 5);
        ode.dJointSetHingeAxis (hinges[i], 0,1,0);
        ode.dJointSetHingeParam (hinges[i],ode.dParamFMax,  8000.0);
        ode.dJointSetFeedback (hinges[i], jfeedbacks[i]);
        stress[i]=0;
    end
    --================================================================
    for i=0,STACKCNT-1  
    do
        stackbodies[i] = ode.dBodyCreate(world);
        ode.dMassSetBox (m, 2.0, 2, 2, 0.6);
        ode.dBodySetMass(stackbodies[i],m);
        stackgeoms[i] =ode.dCreateBox(nil, 2, 2, 0.6);
        ode.dGeomSetBody(stackgeoms[i], stackbodies[i]);
        ode.dBodySetPosition(stackbodies[i], 0,0,8+2*i);
        ode.dSpaceAdd(space, stackgeoms[i]);
    end
    --================================================================
    sliders[0] = ode.dJointCreateSlider ( world,nil);
    ode.dJointAttach         (sliders[0], segbodies[0], nil);
    ode.dJointSetSliderAxis  (sliders[0], 1,0,0);
    ode.dJointSetSliderParam (sliders[0],ode.dParamFMax,  4000.0);
    ode.dJointSetSliderParam (sliders[0],ode.dParamLoStop,   0.0);
    ode.dJointSetSliderParam (sliders[0],ode.dParamHiStop,   0.2);

    sliders[1] = ode.dJointCreateSlider ( world,nil);
    ode.dJointAttach         (sliders[1], segbodies[SEGMCNT-1], nil);
    ode.dJointSetSliderAxis  (sliders[1], 1,0,0);
    ode.dJointSetSliderParam (sliders[1],ode.dParamFMax,  4000.0);
    ode.dJointSetSliderParam (sliders[1],ode.dParamLoStop,   0.0);
    ode.dJointSetSliderParam (sliders[1],ode.dParamHiStop,  -0.2);
    --================================================================
    groundgeom = ode.dCreatePlane(space, 0,0,1,0);
    --================================================================
    for i=0,SEGMCNT-1
    do
        colours[i]=0.0;
    end
    --================================================================
    --drawstuff
    ds.dsSimulationLoop (0,nil,640, 480, func[0]);
    --================================================================
 
    --================================================================ 
    ode.dJointGroupEmpty   (contactgroup);
    ode.dJointGroupDestroy (contactgroup);
    -- First destroy seggeoms, then space, then the world.
    for i=0,SEGMCNT-1
    do
        ode.dGeomDestroy (seggeoms[i]);
    end
    
    for i=0,STACKCNT-1
    do
        ode.dGeomDestroy (stackgeoms[i]);
    end
    --================================================================
    ode.dSpaceDestroy(space);
    ode.dWorldDestroy (world);
    ode.dCloseODE();
    --================================================================
end
--====================================================================
main ()
--====================================================================
