%%
%% Script to the the norm of the gravity vector.
%% @author MARS LAB, University of Minnesota
%% @date Oct 2013
%% Copyright © Regents of the University of Minnesota.  All Rights Reserved
%% 		
%% For a license, please contact:
%% 	
%% 		Prof. Stergios I. Roumeliotis
%% 		Dept. of Computer Science and Engineering
%% 		University of Minnesota
%% 		4-192 KHKH, 200 Union Street SE
%% 		Minneapolis, MN 55455
%% 		(612) 626-7507 fax: (612) 625-0572
%% 		http://www.cs.umn.edu/~stergios
%%
%%
%%
%% You can get lat lon and alt coordinates in degrees, degrees and meters from:
%% http://www.mapcoordinates.net/en
%% Note we convert degrees to radians.
%% Ronald Reagan Washington DC Airport.
format long;
lat = 38.84980183;
lat = lat * (pi / 180);
lon = -77.04257011;
lon = lon * (pi / 180);
alt = 7;
pos_ECEF = latlonalt2ECEF(lat,lon,alt);
norm_of_gravity = norm(g_vector_ECEF(pos_ECEF),2);
sprintf('DC AIRPORT GRAVITY NORM: %10.10f\n',norm_of_gravity)
%% University of Minnesota
format long;
lat = 44.9754336;
lat = lat * (pi / 180);
lon = -93.2362591;
lon = lon * (pi / 180);
alt = 250;
pos_ECEF = latlonalt2ECEF(lat,lon,alt);
norm_of_gravity = norm(g_vector_ECEF(pos_ECEF),2);
sprintf('University of Minnesota GRAVITY NORM: %10.10f\n',norm_of_gravity)
%%
%% Tokyo Big Sight IROS 2013 Conference site
lat = 35.6299028;
lat = lat * (pi / 180);
lon = 139.7939344;
lon = lon * (pi / 180);
alt = 19;
pos_ECEF = latlonalt2ECEF(lat,lon,alt); 
norm_of_gravity = norm(g_vector_ECEF(pos_ECEF),2);
sprintf('Tokyo Big Sight GRAVITY NORM: %10.10f\n',norm_of_gravity)
%%