
--====================================================================
local ffi =require"ffi"
--====================================================================
local ds =require "drawstuff"
local ode = require "ode_ffi"
--====================================================================
local world = nil;-- world
local space  =nil --space;
local ground  =nil --space;
local contactgroup =nil;
local flag = 0;
--====================================================================
local body1 =  nil
local body2 =  nil
local joint1 = nil --
local joint2 = nil
--====================================================================

-- drawstuff simlation loop func
--====================================================================
function simLoop (pause)
    -- print (pause )
    --================================================================
    if (pause == 0 ) -- pause==1 : pause
    then
        local t = 0;
        local step = 0.01;
        local nsteps = 4;
        --============================================================
        for i=0,nsteps-1
        do
            local f = math.sin(t)*10;
            ode.dBodyAddForceAtRelPos(body1,
                                      f, 0, 0, 
                                      0, 0, -0.5); -- at the lower end

            local  g = math.sin(t)*5;
            ode.dBodyAddForceAtRelPos(body2,
                                      0, g, 0, 
                                      0, 0, -0.5); -- at the lower end
            t = t+ step;
            ode.dWorldQuickStep(world, step);
        end 
    end
    --================================================================
    
    -- now we draw everything
    local ngeoms = ode.dSpaceGetNumGeoms(space);
    --================================================================
    for i=0, ngeoms-1
    do
        local  g = ode.dSpaceGetGeom(space, i);
        drawGeom(g);
    end
    --================================================================
    local a11 = ffi.new("dVector3[1]"); 
    local a12 = ffi.new("dVector3[1]"); 
    ode.dJointGetDBallAnchor1(joint1, a11[0]);
    ode.dJointGetDBallAnchor2(joint1, a12[0]);
    ds.dsSetColor(1, 0, 0);
    ds.dsDrawLineD(a11[0], a12[0]);

    local a21 = ffi.new("dVector3[1]"); 
    local a22 = ffi.new("dVector3[1]"); 
    ode.dJointGetDBallAnchor1(joint2, a21[0]);
    ode.dJointGetDBallAnchor2(joint2, a22[0]);
    ds.dsSetColor(0, 1, 0);
    ds.dsDrawLineD(a21[0], a22[0]);
end
--====================================================================

-- drawstuff start func
--====================================================================
function start()
    
    world = ode.dWorldCreate();
    ode.dWorldSetGravity (world,0,0,-9.8);
    ode.dWorldSetDamping(world, 1e-4, 1e-5);

    space = ode.dSimpleSpaceCreate (nil);
    
    body1 = ode.dBodyCreate(world);
    body2 = ode.dBodyCreate(world);

    ode.dBodySetPosition(body1, 0, 0, 3);
    ode.dBodySetPosition(body2, 0, 0, 1);

    local g;
    local  mass = ffi.new("dMass[1]");
    
    g = ode.dCreateBox(space, 0.2, 0.2, 1);
    ode.dGeomSetBody(g, body1);
    ode.dMassSetBox(mass, 1, 0.2, 0.2, 1);
    ode.dBodySetMass(body1, mass);
    
    g = ode.dCreateBox(space, 0.2, 0.2, 1);
    ode.dGeomSetBody(g, body2);
    ode.dMassSetBox(mass, 1, 0.2, 0.2, 1);
    ode.dBodySetMass(body2, mass);

    joint1 = ode.dJointCreateDBall(world, nil);
    ode.dJointAttach(joint1, body1, nil);
    ode.dJointSetDBallAnchor1(joint1, 0, 0, 3.5);
    ode.dJointSetDBallAnchor2(joint1, 0, 0, 4.5);

    joint2 = ode.dJointCreateDBall(world, nil);
    ode.dJointAttach(joint2, body1, body2);
    ode.dJointSetDBallAnchor1(joint2, 0, 0, 2.5);
    ode.dJointSetDBallAnchor2(joint2, 0, 0, 1.5);

    -- initial camera position
    local  xyz = ffi.new("float[3]",3.8966, -2.0614, 4.0300);
    local  hpr = ffi.new("float[3]",153.5, -16.5, 0);
    ds.dsSetViewpoint (xyz,hpr);
end
--====================================================================

--====================================================================
function stop()
    ode.dSpaceDestroy(space);
    ode.dWorldDestroy(world);
end 
--====================================================================

--====================================================================
function drawGeom( g)
    --================================================================
    local gclass = ode.dGeomGetClass(g);
    local pos = ode.dGeomGetPosition(g);
    local rot = ode.dGeomGetRotation(g);
    --================================================================
    if ( g == ode.dSphereClass)
    then 
        ds.dsSetColorAlpha(0, 0.75, 0.5, 1);
        ds.dsSetTexture (ds.DS_CHECKERED);
        ds.dsDrawSphereD(pos, rot, ode.dGeomSphereGetRadius(g));
        
    elseif ( ode.dBoxClass)
    then 
        local lengths = ffi.new("dVector3[1]");
        ds.dsSetColorAlpha(1, 1, 0, 1);
        ds.dsSetTexture (ds.DS_WOOD);
        ode.dGeomBoxGetLengths(g, lengths[0]);
        ds.dsDrawBoxD(pos, rot, lengths[0]);
    end 
    --================================================================
end 
--====================================================================

--====================================================================
function main ()
    --================================================================
    ode.dInitODE();
    --================================================================
    --drawstuff function struct
    local func = ffi.new("dsFunctions[1]");
    func[0].version = ds.DS_VERSION;
    func[0].start = start;
    func[0].step = simLoop;
    func[0].command = nil;      -- nothing func so nil
    func[0].stop  = stop;        
    func[0].path_to_textures = "./textures";

    --drawstuff
    ds.dsSimulationLoop (0,nil,352,288,func[0]);

    -- destroy world
    ode.dCloseODE();
end
--====================================================================
main ()
--====================================================================
