--====================================================================
local ffi = require"ffi"
--====================================================================
local ode = require "ode_ffi"
local ds  = require "drawstuff"
--====================================================================
--from icosahedron_geom.h (ode demo)
--====================================================================

local Sphere_points={ 0.000000,0.000000,-0.300000, 0.217080,-0.157716,-0.134164, -0.082915,-0.255192,-0.134164, -0.268327,0.000000,-0.134164, -0.082915,0.255192,-0.134164, 0.217080,0.157716,-0.134164, 0.082915,-0.255192,0.134164, -0.217080,-0.157716,0.134164, -0.217080,0.157716,0.134164, 0.082915,0.255192,0.134164, 0.268327,0.000000,0.134164, 0.000000,0.000000,0.300000, 0.127597,-0.092703,-0.255196, -0.048737,-0.149999,-0.255196, 0.078861,-0.242703,-0.157721, 0.127597,0.092703,-0.255196, 0.255194,0.000000,-0.157721, -0.157719,0.000000,-0.255195, -0.206457,-0.149999,-0.157721, -0.048737,0.149999,-0.255196, -0.206457,0.149999,-0.157721, 0.078861,0.242703,-0.157721, 0.285317,0.092704,0.000000, 0.285317,-0.092704,0.000000, 0.176336,-0.242705,0.000000, 0.000000,-0.300000,0.000000, -0.176336,-0.242705,0.000000, -0.285317,-0.092704,0.000000, -0.285317,0.092704,0.000000, -0.176336,0.242705,0.000000, 0.000000,0.300000,0.000000, 0.176336,0.242705,0.000000, 0.206457,-0.149999,0.157721, -0.078861,-0.242703,0.157721, -0.255194,0.000000,0.157721, -0.078861,0.242703,0.157721, 0.206457,0.149999,0.157721, 0.157719,0.000000,0.255195, 0.048737,-0.149999,0.255196, -0.127597,-0.092703,0.255196, -0.127597,0.092703,0.255196, 0.048737,0.149999,0.255196 }; 
local Sphere_planes ={ 0.471317,-0.583121,-0.661687,0.283056, 0.187594,-0.577345,-0.794658,0.280252, -0.038547,-0.748789,-0.661687,0.283056, 0.102381,-0.315090,-0.943523,0.283057, 0.700228,-0.268049,-0.661688,0.283056, 0.607060,0.000000,-0.794656,0.280252, 0.700228,0.268049,-0.661688,0.283056, 0.331305,0.000000,-0.943524,0.283057, -0.408939,-0.628443,-0.661686,0.283056, -0.491119,-0.356821,-0.794657,0.280252, -0.724044,-0.194735,-0.661694,0.283057, -0.268034,-0.194737,-0.943523,0.283057, -0.724044,0.194735,-0.661694,0.283057, -0.491119,0.356821,-0.794657,0.280252, -0.408939,0.628443,-0.661686,0.283056, -0.268034,0.194737,-0.943523,0.283057, -0.038547,0.748789,-0.661687,0.283056, 0.187594,0.577345,-0.794658,0.280252, 0.471317,0.583121,-0.661687,0.283056, 0.102381,0.315090,-0.943523,0.283057, 0.904981,-0.268049,-0.330393,0.283056, 0.982246,0.000000,-0.187599,0.280252, 0.992077,0.000000,0.125631,0.283057, 0.904981,0.268049,-0.330393,0.283056, 0.024726,-0.943519,-0.330396,0.283056, 0.303531,-0.934171,-0.187598,0.280251, 0.306568,-0.943519,0.125651,0.283056, 0.534590,-0.777851,-0.330395,0.283056, -0.889698,-0.315092,-0.330386,0.283056, -0.794656,-0.577348,-0.187595,0.280251, -0.802607,-0.583125,0.125648,0.283055, -0.574584,-0.748793,-0.330397,0.283055, -0.574584,0.748793,-0.330397,0.283055, -0.794656,0.577348,-0.187595,0.280251, -0.802607,0.583125,0.125648,0.283055, -0.889698,0.315092,-0.330386,0.283056, 0.534590,0.777851,-0.330395,0.283056, 0.303531,0.934171,-0.187598,0.280251, 0.306568,0.943519,0.125651,0.283056, 0.024726,0.943519,-0.330396,0.283056, 0.889698,-0.315092,0.330386,0.283056, 0.794656,-0.577348,0.187595,0.280251, 0.574584,-0.748793,0.330397,0.283055, 0.802607,-0.583125,-0.125648,0.283055, -0.024726,-0.943519,0.330396,0.283055, -0.303531,-0.934171,0.187598,0.280251, -0.534590,-0.777851,0.330395,0.283056, -0.306568,-0.943519,-0.125651,0.283056, -0.904981,-0.268049,0.330393,0.283056, -0.982246,0.000000,0.187599,0.280252, -0.904981,0.268049,0.330393,0.283056, -0.992077,0.000000,-0.125631,0.283057, -0.534590,0.777851,0.330395,0.283056, -0.303531,0.934171,0.187598,0.280251, -0.024726,0.943519,0.330396,0.283055, -0.306568,0.943519,-0.125651,0.283056, 0.574584,0.748793,0.330397,0.283055, 0.794656,0.577348,0.187595,0.280251, 0.889698,0.315092,0.330386,0.283056, 0.802607,0.583125,-0.125648,0.283055, 0.408939,-0.628443,0.661686,0.283056, 0.491119,-0.356821,0.794657,0.280252, 0.268034,-0.194737,0.943523,0.283057, 0.724044,-0.194735,0.661694,0.283057, -0.471317,-0.583121,0.661687,0.283056, -0.187594,-0.577345,0.794658,0.280252, -0.102381,-0.315090,0.943523,0.283057, 0.038547,-0.748789,0.661687,0.283056, -0.700228,0.268049,0.661688,0.283056, -0.607060,0.000000,0.794656,0.280252, -0.331305,0.000000,0.943524,0.283057, -0.700228,-0.268049,0.661688,0.283056, 0.038547,0.748789,0.661687,0.283056, -0.187594,0.577345,0.794658,0.280252, -0.102381,0.315090,0.943523,0.283057, -0.471317,0.583121,0.661687,0.283056, 0.724044,0.194735,0.661694,0.283057, 0.491119,0.356821,0.794657,0.280252, 0.268034,0.194737,0.943523,0.283057, 0.408939,0.628443,0.661686,0.283056, };
local Sphere_polygons={ 3,14,12,1, 3,12,14,13, 3,2,13,14, 3,13,0,12, 3,16,1,12, 3,12,15,16, 3,5,16,15, 3,12,0,15, 3,18,13,2, 3,13,18,17, 3,3,17,18, 3,17,0,13, 3,20,17,3, 3,17,20,19, 3,4,19,20, 3,19,0,17, 3,21,19,4, 3,19,21,15, 3,5,15,21, 3,15,0,19, 3,23,1,16, 3,16,22,23, 3,10,23,22, 3,22,16,5, 3,25,2,14, 3,14,24,25, 3,6,25,24, 3,24,14,1, 3,27,3,18, 3,18,26,27, 3,7,27,26, 3,26,18,2, 3,29,4,20, 3,20,28,29, 3,8,29,28, 3,28,20,3, 3,31,5,21, 3,21,30,31, 3,9,31,30, 3,30,21,4, 3,32,23,10, 3,23,32,24, 3,6,24,32, 3,24,1,23, 3,33,25,6, 3,25,33,26, 3,7,26,33, 3,26,2,25, 3,34,27,7, 3,27,34,28, 3,8,28,34, 3,28,3,27, 3,35,29,8, 3,29,35,30, 3,9,30,35, 3,30,4,29, 3,36,31,9, 3,31,36,22, 3,10,22,36, 3,22,5,31, 3,38,6,32, 3,32,37,38, 3,11,38,37, 3,37,32,10, 3,39,7,33, 3,33,38,39, 3,11,39,38, 3,38,33,6, 3,40,8,34, 3,34,39,40, 3,11,40,39, 3,39,34,7, 3,41,9,35, 3,35,40,41, 3,11,41,40, 3,40,35,8, 3,37,10,36, 3,36,41,37, 3,11,37,41, 3,41,36,9, };
local Sphere_pointcount =42; -- triangle , #Sphere_points /3
local Sphere_planecount =80; --  ,#Sphere_planes /4 

--==================================================================== 
local Sphere_planesD= ffi.new("double[?]",#Sphere_planes)
local Sphere_pointsD = ffi.new("double[?]",#Sphere_points)
local Sphere_polygonsUI = ffi.new("dTriIndex[?]",#Sphere_polygons)
--====================================================================
for i=0,#Sphere_planes-1
do
    Sphere_planesD[i] = Sphere_planes[i+1];
end
for i=0,#Sphere_points-1
do
    Sphere_pointsD[i] = Sphere_points[i+1];
end
for i=0,#Sphere_polygons-1
do
    Sphere_polygonsUI[i] = Sphere_polygons[i+1];
end
--====================================================================
local NUM =50         -- max number of objects
local DENSITY =(5.0)  -- density of all objects
local GPB =3          -- maximum number of geometries per body
local MAX_CONTACTS =8 -- maximum number of contact points per body
local MAX_FEEDBACKNUM =20
local GRAVITY =(0.5)
local USE_GEOM_OFFSET =1
--====================================================================
local num=0;       -- number of objects in simulation
local nextobj=0;   -- next object to recycle if num==NUM
local world;
local space;
--====================================================================
local obj={}; 
--====================================================================
for i=0,NUM-1
do
    obj[i] ={}
    obj[i].body =nil
    obj[i].geom ={}
    -- for v =0,GPB-1
    -- do
    --     obj[i].geom[v] =nil
    -- end 
end 
--====================================================================

--====================================================================
local contactgroup;
local selected = -1;         -- selected object
local show_aabb = false;     -- show geom AABBs?
local show_contacts = false; -- show contact points?
local random_pos = true;     -- drop objects from random position?
local write_world =false;
--====================================================================
local doFeedback=0;
local feedbacks = {}
--====================================================================
for i=0,MAX_FEEDBACKNUM-1
do
    feedbacks[i] = {}
    feedbacks[i].fb = ffi.new("dJointFeedback[1]");
    feedbacks[i].first =false;
end 
local fbnum=0;
--====================================================================

-- from odemath.h. (has not export func) 
local dMultiply0_331 = nil -- 
local dMultiply0_333 = nil

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
        local RI = ffi.new("dMatrix3[1]");
        ode.dRSetIdentity (RI[0]);
        local ss = ffi.new("dReal[3]",0.05,0.05,0.05);
        --============================================================
        for i=0, numc-1
        do
            contact[i].surface.mode = ode.dContactBounce + ode.dContactSoftCFM;
            contact[i].surface.mu = 1.0/0.0;
            contact[i].surface.mu2 = 0;
            contact[i].surface.bounce = 0.1;
            contact[i].surface.bounce_vel = 0.1;
            contact[i].surface.soft_cfm = 0.01;
            --========================================================
            local c = ode.dJointCreateContact (world,contactgroup,contact);
            ode.dJointAttach (  c
                              , ode.dGeomGetBody(contact[i].geom.g1)
                              , ode.dGeomGetBody(contact[i].geom.g2));
            --========================================================        
            
            if (show_contacts == true)
            then
                ds.dsDrawBoxD (contact[i].geom.pos,RI[0],ss);
            end 
            --========================================================
            
            if ( doFeedback == true
                and ( b1==obj[selected].body ~= 0 or b2==obj[selected].body ~= 0) == true)
            then
                if (fbnum < MAX_FEEDBACKNUM)
                then
                    feedbacks[fbnum].first = (b1==obj[selected].body);
                    ode.dJointSetFeedback (c,feedbacks[fbnum].fb);
                    fbnum=fbnum+1;
                else 
                    fbnum=fbnum+1;
                end 
            end
            --========================================================     
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
function start()
    ode.dAllocateODEDataForThread(ode.dAllocateMaskAll);
    --================================================================
    local xyz = ffi.new("float[3]",2.1640,-1.3079,1.7600)
    local hpr = ffi.new("float[3]",125.5000,-17.0000,0.0000)
    ds.dsSetViewpoint (xyz,hpr);
    --================================================================
    print([[
    To drop another object, press:
      b for box.
      s for sphere.
      c for capsule.
      y for cylinder.
      v for a convex object.
      x for a composite object.
      
    To select an object, press "space".
    To disable the selected object, press "d".
    To enable the selected object, press "e".
    To dump transformation data for the selected object, press "p".
    To show joint feedbacks of selected object, press "f".
    
    To toggle showing the geom AABBs, press "a".
    To toggle showing the contact points, press "t".
    To toggle dropping from random position/orientation, press "r".
        ]])
end
--====================================================================
function drawGeom ( g,pos, R, show_aabb)

    if (g == nil)then return;end
    --================================================================
    if (pos == 0)then pos = ode.dGeomGetPosition (g);end
    if (R == 0)  then R   = ode.dGeomGetRotation (g);end 
    --================================================================
    
    local type_ = ode.dGeomGetClass (g);
    --================================================================
    if (type_ == ode.dBoxClass)
    then
        local sides = ffi.new("dVector3[1]");
        ode.dGeomBoxGetLengths (g,sides[0]);
        ds.dsDrawBoxD(pos,R,sides[0]);
        --============================================================
    elseif (type_ == ode.dSphereClass)
    then
        ds.dsDrawSphereD (pos,R,ode.dGeomSphereGetRadius (g));
        --============================================================
    elseif (type_ == ode.dCapsuleClass)
    then
        local radius=ffi.new("dReal[1]");
        local length=ffi.new("dReal[1]");
        --============================================================
        ode.dGeomCapsuleGetParams (g,radius,length);
        ds.dsDrawCapsuleD(pos,R,length[0],radius[0]);
        --============================================================
    -- Convex Object
    elseif (type_ == ode.dConvexClass) 
    then
        ds.dsDrawConvexD( pos,R
                        , Sphere_planesD
                        , Sphere_planecount
                        , Sphere_pointsD
                        , Sphere_pointcount
                        , Sphere_polygonsUI);
        
        --============================================================
    elseif (type_ == ode.dCylinderClass)
    then
        local radius=ffi.new("dReal[1]");
        local length=ffi.new("dReal[1]");
        ode.dGeomCylinderGetParams (g,radius,length);
        local lengthF = ffi.new("float[1]",length[0])
        local radiusF = ffi.new("float[1]",radius[0])
        ds.dsDrawCylinderD(pos,R,lengthF[0],radiusF[0]);
        --============================================================
    elseif (type_ == ode.dGeomTransformClass)
    then
        local g2 = ode.dGeomTransformGetGeom (g);
        local pos2 = ode.dGeomGetPosition (g2);
        local R2 = ode.dGeomGetRotation (g2);
        --============================================================
        local actual_pos = ffi.new("dVector3[1]"); -- dReal[3]
        local actual_R = ffi.new("dMatrix3[1]");   -- dReal[12]
        ode._dMultiply0_331 (actual_pos[0],R,pos2);
        actual_pos[0][0] = actual_pos[0][0]+pos[0];
        actual_pos[0][1] = actual_pos[0][1]+pos[1];
        actual_pos[0][2] = actual_pos[0][2]+pos[2];
        ode._dMultiply0_333 (actual_R[0],R,R2);
        drawGeom (g2,actual_pos[0],actual_R[0],0);
        --============================================================
    end
    --================================================================
    
    --================================================================
    if (show_aabb ==true)
    then 
        -- draw the bounding box for this geom
        local aabb =ffi.new("dReal[6]");
        ode.dGeomGetAABB (g,aabb);
        --============================================================
        local bbpos=ffi.new("dVector3[1]");
        --============================================================
        for i=0, 3-1
        do
            bbpos[0][i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
        end 
        --============================================================
        local bbsides = ffi.new("dVector3[1]");
        --============================================================
        for i=0, 3-1
        do
            bbsides[0][i] = aabb[i*2+1] - aabb[i*2];
        end 
        --============================================================
        local RI = ffi.new("dMatrix3[1]");
        ode.dRSetIdentity (RI[0]);
        --============================================================
        ds.dsSetColorAlpha (1,0,0,0.5);
        ds.dsDrawBoxD (bbpos[0],RI[0],bbsides[0]);
    end 
    --================================================================
end
--====================================================================
function simLoop(pause)
    
    ds.dsSetColor (0,0,2);
    if ( pause ==0)
    then 
        --============================================================
        local callBack = ffi.cast ( "dNearCallback",nearCallBack_checkSpace)
        ode.dSpaceCollide(space,nil,callBack);
        callBack:free();
        --============================================================
        ode.dWorldQuickStep (world,0.02);
    end 
    --================================================================

    --================================================================
    if (write_world == true)
    then 
        ode._dWorldExportDIF(world,"","state.dif", "wt");
        print("save file as state.dif");
        write_world = false;
    end 

    --================================================================
    if (doFeedback == true)
    then 
        if (fbnum>MAX_FEEDBACKNUM)
        then
            print("joint feedback buffer overflow!");
        else
            local sum = ffi.new("dVector3[1]");
            sum[0][0]=0
            sum[0][1]=0
            sum[0][2]=0
            --========================================================
            for i=0,fbnum-1 
            do
                local f =0;--false
                if ( feedbacks[i].first  ~=0)
                then
                    f = feedbacks[i].fb[0].f1
                else
                    f = feedbacks[i].fb[0].f2;
                end
                --====================================================
                sum[0][0] = sum[0][0] + f[0];
                sum[0][1] = sum[0][1] + f[1];
                sum[0][2] = sum[0][2] + f[2];
            end
            --========================================================
            print(string.format("Sum: %f %f %f", sum[0][0], sum[0][1], sum[0][2]));
            local m =ffi.new("dMass[1]");
            ode.dBodyGetMass(obj[selected].body, m);
            print(string.format("Object G=%f", GRAVITY*m[0].mass));
            --========================================================
        end
        doFeedback = false;
        fbnum = 0;
    end 
    --================================================================

    -- remove all contact joints
    ode.dJointGroupEmpty (contactgroup);

    ds.dsSetColor (1,1,0);
    ds.dsSetTexture (ds.DS_WOOD);
    --================================================================
    for i=0,num-1
    do 
        for j=0,GPB-1
        do
            if (i==selected)
            then 
                ds.dsSetColor (0,0.7,1);
            elseif (ode.dBodyIsEnabled (obj[i].body) ) 
            then
                ds.dsSetColor (1,0.8,0);
            else 
                ds.dsSetColor (1,1,0);
            end
            drawGeom (obj[i].geom[j],0,0,show_aabb);
        end
    end
    --================================================================
end 
--====================================================================
function command (cmd)

    local i ,j,k;
    local sides = ffi.new("dReal[3]");
    local m = ffi.new("dMass[1]");
    local setBody =0;
    --================================================================
    local str = string.char(cmd)
    cmd = string.lower(str)
    --================================================================
    if (   cmd == 'b' 
        or cmd == 's' 
        or cmd == 'c' 
        or cmd == 'x' 
        or cmd == 'y' 
        or cmd == 'v')
    then 
        setBody = 0;
        --============================================================
        if (num < NUM)
        then 
            i = num;
            num=num+1;
        else 
            i = nextobj;
            nextobj=nextobj+1;
            --========================================================
            if (nextobj >= NUM)
            then
                nextobj = 0;
            end
            --========================================================
            if (obj[i].body~=0 )
            then
                ode.dBodyDestroy (obj[i].body);
                obj[i].body = nil
            end
            --========================================================
            for k=0, GPB-1
            do
                if (obj[i].geom[k] ~= nil)
                then
                    ode.dGeomDestroy (obj[i].geom[k]);
                    obj[i].geom[k] =nil
                end 
                
            end
            --========================================================
            --memset (&obj[i],0,sizeof(obj[i]));
        end
        --============================================================
        obj[i].body = ode.dBodyCreate (world);
        for k=0, 3-1
        do
            sides[k] = ode.dRandReal()*0.5+0.1;
        end
        --============================================================
        local R =ffi.new("dMatrix3[1]");
        --============================================================
        if (random_pos ==true) 
        then
            ode.dBodySetPosition( obj[i].body
                                , ode.dRandReal()*2-1
                                , ode.dRandReal()*2-1,ode.dRandReal()+2);
            ode.dRFromAxisAndAngle( R[0]
                                  , ode.dRandReal()*2.0-1.0
                                  , ode.dRandReal()*2.0-1.0
                                  , ode.dRandReal()*2.0-1.0
                                  , ode.dRandReal()*10.0-5.0);
            --========================================================
        else 
            local maxheight = 0;
            for k=0, num-1
            do
                local pos = ode.dBodyGetPosition (obj[k].body);
                if (pos[2] > maxheight)
                then 
                    maxheight = pos[2];
                end 
            end
            --========================================================
            ode.dBodySetPosition (obj[i].body, 0,0,maxheight+1);
            ode.dRSetIdentity (R[0]);
        end
        --============================================================
        ode.dBodySetRotation (obj[i].body,R[0]);
        --ode.dBodySetData (obj[i].body,(void*) i);
        --============================================================
        
        --============================================================
        if (cmd == 'b') 
        then 
            ode.dMassSetBox (m,DENSITY,sides[0],sides[1],sides[2]);
            obj[i].geom[0] = ode.dCreateBox (space,sides[0],sides[1],sides[2]);
            --========================================================
        elseif (cmd == 'c') 
        then
            sides[0] =sides[0]* 0.5;
            ode.dMassSetCapsule (m,DENSITY,3,sides[0],sides[1]);
            obj[i].geom[0] = ode.dCreateCapsule (space,sides[0],sides[1]);
            --========================================================
        elseif (cmd == 'v') 
        then
            ode.dMassSetBox (m,DENSITY,0.25,0.25,0.25);
            obj[i].geom[0] = ode.dCreateConvex (  space
                                                , Sphere_planesD
                                                , Sphere_planecount
                                                , Sphere_pointsD
                                                , Sphere_pointcount
                                                , Sphere_polygonsUI );
            --========================================================
        elseif (cmd == 'y')
        then 
            ode.dMassSetCylinder (m,DENSITY,3,sides[0],sides[1]);
            obj[i].geom[0] = ode.dCreateCylinder (space,sides[0],sides[1]);
            --========================================================
        elseif (cmd == 's') 
        then 
            sides[0] =sides[0]*0.5;
            ode.dMassSetSphere (m,DENSITY,sides[0]);
            obj[i].geom[0] = ode.dCreateSphere (space,sides[0]);
            --========================================================
        elseif (cmd == 'x' and USE_GEOM_OFFSET ~= 0) 
        then 
            setBody = 1;
            local m2 =ffi.new("dMass[1]");
            ode.dMassSetZero (m);
            ode.dMassSetZero (m2);
            ----========================================================
            local dpos ={}
            ----========================================================
            local drot = ffi.new("dMatrix3[?]",GPB);
            
            -- set random delta positions
            for j=0,GPB-1
            do
                dpos[j]={}
                for k=0 ,3-1
                do
                    dpos[j][k] = ode.dRandReal()*0.3-0.15;
                end
            end
            --========================================================

            for k=0, GPB-1
            do
                if (k==0) 
                then
                    local radius = ode.dRandReal()*0.25+0.05;
                    obj[i].geom[k] = ode.dCreateSphere (space,radius);
                    ode.dMassSetSphere (m2,DENSITY,radius);
                    --================================================
                elseif (k==1) 
                then
                    obj[i].geom[k] = ode.dCreateBox (space,sides[0],sides[1],sides[2]);
                    ode.dMassSetBox (m2,DENSITY,sides[0],sides[1],sides[2]);
                    --================================================
                else
                    local radius = ode.dRandReal()*0.1+0.05;
                    local length = ode.dRandReal()*1.0+0.1;
                    obj[i].geom[k] = ode.dCreateCapsule (space,radius,length);
                    ode.dMassSetCapsule (m2,DENSITY,3,radius,length);
                    --================================================
                end
                --====================================================
                ode.dRFromAxisAndAngle ( drot[k]
                                       , ode.dRandReal()*2.0-1.0
                                       , ode.dRandReal()*2.0-1.0
                                       , ode.dRandReal()*2.0-1.0
                                       , ode.dRandReal()*10.0-5.0);
                ode.dMassRotate (m2,drot[k]);
                --====================================================
                ode.dMassTranslate (m2,dpos[k][0],dpos[k][1],dpos[k][2]);
                -- add to the total mass
                ode.dMassAdd (m,m2);
            end  
            --========================================================
            
            --========================================================
            for k=0, GPB-1
            do
                ode.dGeomSetBody (obj[i].geom[k],obj[i].body);
                ode.dGeomSetOffsetPosition ( obj[i].geom[k]
                                           , dpos[k][0]-m[0].c[0]
                                           , dpos[k][1]-m[0].c[1]
                                           , dpos[k][2]-m[0].c[2]);
                ode.dGeomSetOffsetRotation(obj[i].geom[k], drot[k]);
            end
            --========================================================
            ode.dMassTranslate (m,-m[0].c[0],-m[0].c[1],-m[0].c[2]);
            ode.dBodySetMass (obj[i].body,m);
            --========================================================
        elseif (cmd == 'x')
        then
            local g2 ={};  -- encapsulated geometries
            local dpos = {}-- delta-positions for encapsulated geometries
            --========================================================
            -- start accumulating masses for the encapsulated geometries
            local  m2 = ffi.new("dMass[1]");
            ode.dMassSetZero (m);
            ode.dMassSetZero (m2);
            
            -- set random delta positions
            for j=0,GPB-1
            do
                dpos[j] = {}
                for k=0,3-1
                do
                    dpos[j][k] = ode.dRandReal()*0.3-0.15;
                end
            end
            --========================================================
            for k=0, GPB-1
            do
                obj[i].geom[k] = ode.dCreateGeomTransform (space);
                ode.dGeomTransformSetCleanup (obj[i].geom[k],1);
                if (k==0)
                then
                    local radius = ode.dRandReal()*0.25+0.05;
                    g2[k] = ode.dCreateSphere (nil,radius);
                    ode.dMassSetSphere (m2,DENSITY,radius);
                    --================================================
                elseif (k==1)
                then
                    g2[k] = ode.dCreateBox (nil,sides[0],sides[1],sides[2]);
                    ode.dMassSetBox (m2,DENSITY,sides[0],sides[1],sides[2]);
                    --================================================
                else 
                
                    local radius = ode.dRandReal()*0.1+0.05;
                    local length = ode.dRandReal()*1.0+0.1;
                    g2[k] = ode.dCreateCapsule (nil,radius,length);
                    ode.dMassSetCapsule (m2,DENSITY,3,radius,length);
                end
                --====================================================
                
                ode.dGeomTransformSetGeom (obj[i].geom[k],g2[k]);
                
                -- set the transformation (adjust the mass too)
                ode.dGeomSetPosition (g2[k],dpos[k][0],dpos[k][1],dpos[k][2]);
                local Rtx =ffi.new("dMatrix3[1]");
                --====================================================
                
                ode.dRFromAxisAndAngle( Rtx[0]
                                      , ode.dRandReal()*2.0-1.0
                                      , ode.dRandReal()*2.0-1.0
                                      , ode.dRandReal()*2.0-1.0
                                      , ode.dRandReal()*10.0-5.0);
                                  
                ode.dGeomSetRotation (g2[k],Rtx[0]);
                ode.dMassRotate (m2,Rtx[0]);
                --====================================================
                -- Translation *after* rotation
                ode.dMassTranslate (m2,dpos[k][0],dpos[k][1],dpos[k][2]);
--
                -- add to the total mass
                ode.dMassAdd (m,m2);
            end
            --========================================================
            
            --========================================================
            for k=0, GPB-1
            do
                ode.dGeomSetPosition (g2[k],
                                      dpos[k][0]-m[0].c[0],
                                      dpos[k][1]-m[0].c[1],
                                      dpos[k][2]-m[0].c[2]);
            end
            --========================================================
            ode.dMassTranslate (m,-m[0].c[0],-m[0].c[1],-m[0].c[2]);
        end
        --============================================================
        if (setBody )
        then
            for k=0, GPB-1
            do
                if (obj[i].geom[k])
                then 
                    ode.dGeomSetBody (obj[i].geom[k],obj[i].body);
                end
            end 
        end
        --============================================================
        ode.dBodySetMass (obj[i].body,m);
    end
        
    --================================================================
    if ( cmd =='a')
    then
        show_aabb = (show_aabb ==false) 
        --============================================================
    elseif ( cmd =='t')
    then
        show_contacts = (show_contacts ==false) 
        --============================================================
    elseif ( cmd =='r')
    then
        random_pos = (random_pos ==false) 
        --============================================================
    elseif ( cmd ==' ')
    then
        selected=selected +1;
        if (selected >= num)
        then
            selected = 0;
        end 
        if (selected < 0)
        then 
            selected = 0;
        end 
        --============================================================
    elseif (   cmd == 'd' 
           and selected >= 0
           and selected < num ) 
    then 
        ode.dBodyDisable (obj[selected].body);
        --============================================================
    elseif (    cmd == 'e' 
            and selected >= 0 
            and selected < num )
    then
        ode.dBodyEnable (obj[selected].body);
        --============================================================
    elseif (    cmd == 'f' 
            and selected >= 0 
            and selected < num )
    then
        if (ode.dBodyIsEnabled(obj[selected].body)~=0)
        then
            doFeedback = true;
        end 
        --============================================================
    elseif (    cmd == 'p'
            and selected >= 0 )
    then
        local pos = ode.dGeomGetPosition(obj[selected].geom[0]);
        local rot = ode.dGeomGetRotation(obj[selected].geom[0]);
        print( string.format("POSITION:\n\t[%f,%f,%f]\n\n",pos[0],pos[1],pos[2]));
        print( string.format("ROTATION:\n\t[%f,%f,%f,%f]\n\t[%f,%f,%f,%f]\n\t[%f,%f,%f,%f]\n\n"
             , rot[0],rot[1],rot[2],rot[3]
             , rot[4],rot[5],rot[6],rot[7]
             , rot[8],rot[9],rot[10],rot[11]));
        --============================================================ 
    elseif ( cmd == '1')
    then
        write_world =true;
        --============================================================
    end 
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
    func[0].stop  = nil;       
    func[0].path_to_textures = "./textures";
    --================================================================
   
    -- create world
    ode.dInitODE2(0);
    world = ode.dWorldCreate();
    space = ode.dHashSpaceCreate (nil);
    contactgroup = ode.dJointGroupCreate (0);
    ode.dWorldSetGravity (world,0,0,-GRAVITY);
    ode.dWorldSetCFM (world,1e-5);
    ode.dWorldSetAutoDisableFlag (world,1);
    ode.dWorldSetAutoDisableAverageSamplesCount( world, 10 );
    --================================================================
    --ode.dWorldSetLinearDamping(world, 0.00001);
    --ode.dWorldSetAngularDamping(world, 0.005);
    --ode.dWorldSetMaxAngularSpeed(world, 200);
    --
    --ode.dWorldSetContactMaxCorrectingVel (world,0.1);
    --ode.dWorldSetContactSurfaceLayer (world,0.001);
    
    --================================================================
    ode.dCreatePlane (space,0,0,1,0);
    --================================================================
    
    --================================================================
    local threading = ode.dThreadingAllocateMultiThreadedImplementation();
    local pool = ode.dThreadingAllocateThreadPool(4, 0, ode.dAllocateFlagBasicData, nil);
    ode.dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
    ode.dWorldSetStepThreadingImplementation(world, ode.dThreadingImplementationGetFunctions(threading), threading);
    --================================================================
    
    --================================================================
    --drawstuff
    ds.dsSimulationLoop (0,nil,640, 480, func[0]);
    --================================================================
 
    ode.dThreadingImplementationShutdownProcessing(threading);
    ode.dThreadingFreeThreadPool(pool);
    ode.dWorldSetStepThreadingImplementation(world, nil, nil);
    ode.dThreadingFreeImplementation(threading);

    ode.dJointGroupDestroy (contactgroup);
    ode.dSpaceDestroy (space);
    ode.dWorldDestroy (world);
    ode.dCloseODE();
    --================================================================
end
--====================================================================


-- from odemath.h. (has not export func) --> have exported dll
--====================================================================
function _dCalcVectorDot3(a, b, step_a,step_b)
  return a[0] * b[0] + a[step_a] * b[step_b] + a[2 * step_a] * b[2 * step_b];
end
--====================================================================
function dCalcVectorDot3_41 (a, b)
    return _dCalcVectorDot3(a,b,4,1); 
end
--====================================================================
function dSubtractVectors3(a,b)
    local res ={}
    res[0] = a[0] - b[0];
    res[1] = a[1] - b[1];
    res[2] = a[2] - b[2];
    return res;
end
--====================================================================
function dCalcVectorLength3(a)
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
end
--====================================================================
function dCalcPointsDistance3(a,b)
    local tmp = dSubtractVectors3( a,b);
    local res = dCalcVectorLength3(tmp);
    return res;
end
--====================================================================
function dCalcVectorDot3(a, b) 
    return _dCalcVectorDot3(a,b,1,1); 
end
--====================================================================
function dMultiplyHelper0_331(res,a, b)
    local res_0 = dCalcVectorDot3(a, b);
    local res_1 = dCalcVectorDot3(a + 4, b);
    local res_2 = dCalcVectorDot3(a + 8, b);
    res[0] = res_0; res[1] = res_1; res[2] = res_2;
end
--====================================================================
function dMultiplyHelper1_331(res,a,b)
    local res_0 = dCalcVectorDot3_41(a, b);
    local res_1 = dCalcVectorDot3_41(a + 1, b);
    local res_2 = dCalcVectorDot3_41(a + 2, b);
    res[0] = res_0; res[1] = res_1; res[2] = res_2;
end
--====================================================================
function dMultiplyHelper0_133(res,a,b)
    dMultiplyHelper1_331(res, b, a);
end
--====================================================================
function dMultiply0_331(res, a, b)
    dMultiplyHelper0_331(res, a, b);
end  
--====================================================================
function dMultiply0_333(res,a,b)
    dMultiplyHelper0_133(res + 0, a + 0, b);
    dMultiplyHelper0_133(res + 4, a + 4, b);
    dMultiplyHelper0_133(res + 8, a + 8, b);
end
--====================================================================

--====================================================================
main ()
--====================================================================
