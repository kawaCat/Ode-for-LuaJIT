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
local theta = math.pi / 4;
local ratio = 1
local speed = 5
local rho_1 = 1
local rho_2 = 1
local backlash = 0.1;
local mode = 0;

--====================================================================
local body1=nil 
local body2=nil
local geom1=nil 
local geom2=nil
local hinge1=nil
local hinge2=nil
local transmission=nil
local feedback=ffi.new("dJointFeedback[1]")
--====================================================================
local cos = math.cos
local sin = math.sin
--====================================================================
local t = 0;

--====================================================================
function setup()
    
    local R = ffi.new("dMatrix3[1]");
    --================================================================
    if ( mode ==0)
    then
        ode.dBodySetPosition(body1, 1, 0, 1);
        ode.dBodySetPosition(body2, -1, 0, 1);

        ode.dRSetIdentity (R[0]);
        ode.dBodySetRotation (body1, R[0]);
        ode.dBodySetRotation (body2, R[0]);

        ode.dJointSetHingeAnchor(hinge2, -1, 0, 1);
        ode.dJointSetHingeAxis(hinge2, 0, 0, 1);

        ode.dJointSetHingeAnchor(hinge1, 1, 0, 1);
        ode.dJointSetHingeAxis(hinge1, 0, 0, 1);

        ode.dJointSetTransmissionMode(transmission, ode.dTransmissionParallelAxes);
        ode.dJointSetTransmissionRatio(transmission, ratio);
        ode.dJointSetTransmissionAnchor1(transmission, 1, 0, 1);
        ode.dJointSetTransmissionAnchor2(transmission, -1, 0, 1);
        ode.dJointSetTransmissionAxis(transmission, 0, 0, 1);
        --============================================================
    elseif ( mode ==1)
    then
        ode.dBodySetPosition(body1, 1, 0, 1);
        ode.dBodySetPosition(body2, -1, 0, 2);

        ode.dRSetIdentity (R[0]);
        ode.dBodySetRotation (body1, R[0]);

        ode.dRFromZAxis (R[0], cos(theta), 0, sin(theta));
        ode.dBodySetRotation (body2, R[0]);

        ode.dJointSetHingeAnchor(hinge2, -1, 0, 2);
        ode.dJointSetHingeAxis(hinge2, cos(theta), 0, sin(theta));

        ode.dJointSetHingeAnchor(hinge1, 1, 0, 1);
        ode.dJointSetHingeAxis(hinge1, 0, 0, 1);
    
        ode.dJointSetTransmissionMode(transmission, ode.dTransmissionIntersectingAxes);
        ode.dJointSetTransmissionAnchor1(transmission, 1, 0, 1);
        ode.dJointSetTransmissionAnchor2(transmission, -1, 0, 2);
        ode.dJointSetTransmissionAxis1(transmission, 0, 0, -1);
        ode.dJointSetTransmissionAxis2(transmission, cos(theta), 0, sin(theta));
        --============================================================
    elseif( mode ==2)
    then
        ode.dBodySetPosition(body1, 2, 0, 1);
        ode.dBodySetPosition(body2, -2, 0, 1);

        ode.dRSetIdentity (R[0]);
        ode.dBodySetRotation (body1, R[0]);
        ode.dBodySetRotation (body2, R[0]);

        ode.dJointSetHingeAnchor(hinge2, -2, 0, 1);
        ode.dJointSetHingeAxis(hinge2, 0, 0, 1);

        ode.dJointSetHingeAnchor(hinge1, 2, 0, 1);
        ode.dJointSetHingeAxis(hinge1, 0, 0, 1);

        ode.dJointSetTransmissionMode(transmission, ode.dTransmissionChainDrive);
        ode.dJointSetTransmissionAnchor1(transmission, 2, 0, 1);
        ode.dJointSetTransmissionAnchor2(transmission, -2, 0, 1);
        ode.dJointSetTransmissionRadius1(transmission, rho_1);
        ode.dJointSetTransmissionRadius2(transmission, rho_2);
        ode.dJointSetTransmissionAxis(transmission, 0, 0, 1);
    end 
    --================================================================
    
    ode.dJointSetTransmissionBacklash(transmission, backlash);

    ode.dJointSetHingeParam(hinge2, ode.dParamVel, speed);
    ode.dJointSetHingeParam(hinge2, ode.dParamFMax, 50);

    ode.dJointSetHingeParam(hinge1, ode.dParamVel, 0);
    ode.dJointSetHingeParam(hinge1, ode.dParamFMax, 2);

    ode.dBodySetLinearVel(body1, 0, 0, 0);
    ode.dBodySetLinearVel(body2, 0, 0, 0);
    ode.dBodySetAngularVel(body1, 0, 0, 0);
    ode.dBodySetAngularVel(body2, 0, 0, 0);
end 


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
function start()
    
    local mass =ffi.new("dMass[1]");
    --================================================================
    
    world = ode.dWorldCreate();
    ode.dWorldSetGravity (world,0,0,-9.8);
    ode.dWorldSetERP(world, 0.2);
    space = ode.dSimpleSpaceCreate (nil);
    --================================================================
    
    body1 = ode.dBodyCreate(world);
    body2 = ode.dBodyCreate(world);
    ode.dBodySetFiniteRotationMode(body1, 1);
    ode.dBodySetFiniteRotationMode(body2, 1);
    --================================================================
    
    geom1 = ode.dCreateCylinder(space, 0.2, 0.5);
    ode.dGeomSetBody(geom1, body1);
    ode.dMassSetCylinder(mass, 100, 3, 0.2, 0.5);
    ode.dBodySetMass(body1, mass);
    --================================================================
    geom2 = ode.dCreateCylinder(space, 0.2, 0.5);
    ode.dGeomSetBody(geom2, body2);
    ode.dMassSetCylinder(mass, 100, 3, 0.2, 0.5);
    ode.dBodySetMass(body2, mass);
    --================================================================
    
    hinge1 = ode.dJointCreateHinge(world, nil);
    ode.dJointAttach(hinge1, body1, nil);
    --================================================================
    hinge2 = ode.dJointCreateHinge(world, nil);
    ode.dJointAttach(hinge2, body2, nil);
    --================================================================
    
    transmission = ode.dJointCreateTransmission(world, nil);
    ode.dJointAttach(transmission, body1, body2);
    ode.dJointSetFeedback(transmission, feedback);
    --================================================================
    setup();
    
    --================================================================
    local xyz = ffi.new("float[3]",1.15,-2.78,4.1);
    local hpr = ffi.new("float[3]",105,-45.5,0);
    ds.dsSetViewpoint (xyz,hpr);  
    --================================================================
    
    print ([[
    The green wheel is driving the red one. To control it use the following:
       '[' : decrease wheel ratio
       ']' : increase wheel ratio
       ',' : decrease driving wheel speed
       '.' : increase driving wheel speed
       '-' : decrease backlash
       '=' : increase backlash
       '1' : switch to parallel axes gears mode
       '2' : switch to intersecting axes gears mode
       '3' : switch to chain (or belt) mode
    ]] )
end
--====================================================================
function stop()
    ode.dSpaceDestroy(space);
    ode.dWorldDestroy(world);
end 
--====================================================================
function drawGeom(g)
    local gclass = ode.dGeomGetClass(g);
    local pos = ode.dGeomGetPosition(g);
    local rot = ode.dGeomGetRotation(g);
    --================================================================
    if (gclass == ode.dCylinderClass)
    then 
        local length =ffi.new("dReal[1]")
        local radius =ffi.new("dReal[1]")
        --============================================================
        if (g == geom1)
        then
            ds.dsSetColorAlpha(1, 0, 0, 1);
        else
            ds.dsSetColorAlpha(0, 1, 0, 1);
        end 
        --============================================================
        ds.dsSetTexture (ds.DS_WOOD);
        ode.dGeomCylinderGetParams(g, radius, length);
        ds.dsDrawCylinderD(pos, rot, length[0], radius[0]);
    end
end
--====================================================================
function simLoop(pause)
    
    if (pause ==0)
    then
        local step = 0.003;
        local nsteps = 4;
        
        for i=0,nsteps-1
        do
            ode.dWorldQuickStep(world, step);
        end
    end
    --================================================================
    
    -- now we draw everything
    local ngeoms = ode.dSpaceGetNumGeoms(space);
    for  i=0,ngeoms-1
    do
        local g = ode.dSpaceGetGeom(space, i);
        drawGeom(g);
    end 
    --================================================================

    local R_1 = ode.dGeomGetRotation(geom1);
    local R_2 = ode.dGeomGetRotation(geom2);
    local c_1 =ffi.new("dVector3[1]"); -- typedef dReal dVector3[3]
    local c_2 =ffi.new("dVector3[1]");  
    local a_1 =ffi.new("dVector3[1]");  
    local a_2 =ffi.new("dVector3[1]");
    ode.dJointGetTransmissionContactPoint1(transmission, c_1[0]);
    ode.dJointGetTransmissionContactPoint2(transmission, c_2[0]);
    ode.dJointGetTransmissionAnchor1(transmission, a_1[0]);
    ode.dJointGetTransmissionAnchor2(transmission, a_2[0]);

    -- ??
    -- c_1[0] : == &dVector3 == *dReal[3]
    ------------------
    -- c_1[0][0]:-- x
    -- c_1[0][1]:-- y
    -- c_1[0][2]:-- z

    ds.dsSetColorAlpha(1, 0, 0, 0.5);
    ds.dsDrawCylinderD(a_1[0], R_1, 0.05, ode._dCalcPointsDistance3(c_1[0], a_1[0]));
    ds.dsSetColorAlpha(0, 1, 0, 0.5);
    ds.dsDrawCylinderD(a_2[0], R_2, 0.05, ode._dCalcPointsDistance3(c_2[0], a_2[0]));

    ds.dsSetColorAlpha(1, 0, 0, 0.5);
    ds.dsDrawSphereD (c_1[0], R_1, 0.05);
    ds.dsDrawSphereD (c_2[0], R_1, 0.05);

    --================================================================
    ds.dsSetColorAlpha(1, 1, 0, 0.5);
    if (mode == ode.dTransmissionChainDrive ) 
    then
        ds.dsDrawLineD(c_1[0], c_2[0]);
    end 
    --================================================================
end 
--====================================================================
function command (cmd)
    --================================================================
    if (cmd == string.byte'[') 
    then 
        if (mode == ode.dTransmissionParallelAxes) 
        then 
            if (ratio > 0.125) 
            then
                ratio =ratio * 0.5;
            end
            --========================================================
        elseif (mode == ode.dTransmissionIntersectingAxes) 
        then 
            if (theta > 0.1) 
            then
                theta =theta- 0.1;
            end
            --========================================================
        elseif (mode == ode.dTransmissionChainDrive) 
        then 
            if (rho_2 > 0.125)
            then 
                rho_2 =rho_2 / 2;
            end
            --========================================================
        end
        setup();
        --============================================================
    elseif(cmd == string.byte']')
    then
        if (mode == ode.dTransmissionParallelAxes) 
        then 
            if (ratio < 8) 
            then
                ratio =ratio * 2;
            end
            --========================================================
        elseif (mode == ode.dTransmissionIntersectingAxes) 
        then 
            if (theta < 0.9) 
            then
                theta =theta + 0.1;
            end
            --========================================================
        elseif (mode == ode.dTransmissionChainDrive) 
        then 
            if (rho_2 < 2)
            then 
                rho_2 =rho_2 * 2;
            end
            --========================================================
        end
        setup();
        --============================================================
    elseif(cmd == string.byte'.')
    then 
        speed =speed+ 5;
        ode.dJointSetHingeParam(hinge2, ode.dParamVel, speed);
        --============================================================
    elseif(cmd == string.byte'.')
    then 
        speed =speed- 5;
        ode.dJointSetHingeParam(hinge2, ode.dParamVel, speed);
        --============================================================
    elseif(cmd == string.byte'/')
    then 
        if (ode.dJointGetHingeParam(hinge2, ode.dParamFMax) > 0)
        then
            ode.dJointSetHingeParam(hinge2, ode.dParamFMax, 0);
        else 
            ode.dJointSetHingeParam(hinge2, ode.dParamFMax, 50);
        end
        --============================================================
    elseif(cmd == string.byte'-')
    then 
        backlash = backlash -0.1;
        ode.dJointSetTransmissionBacklash(transmission, backlash);
        --============================================================
    elseif(cmd == string.byte'=')
    then 
        backlash = backlash +0.1;
        ode.dJointSetTransmissionBacklash(transmission, backlash);
        --============================================================
    elseif(cmd == string.byte'1')
    then 
        mode = ode.dTransmissionParallelAxes;
        setup();
        --============================================================
    elseif(cmd == string.byte'2')
    then 
        mode = ode.dTransmissionIntersectingAxes;
        setup();
       --============================================================
    elseif(cmd == string.byte'3')
    then 
        mode = ode.dTransmissionChainDrive;
        setup();
        --============================================================
    end
    --================================================================
end 
--====================================================================
function main ()
    --================================================================
    --drawstuff function struct
    local func = ffi.new("dsFunctions[1]");
    func[0].version = ds.DS_VERSION;
    func[0].start = start;
    func[0].step = simLoop;
    func[0].command = command;     
    func[0].stop  = stop;       
    func[0].path_to_textures = "./textures";
    --================================================================
   
    -- create world
    ode.dInitODE();
    --================================================================
    --drawstuff
    ds.dsSimulationLoop (0,nil,640, 480, func[0]);
    --================================================================
 
    ode.dCloseODE();
    --================================================================
end
--====================================================================
main ()
--====================================================================
