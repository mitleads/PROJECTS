import numpy as np
from SUAVE.Core import Units
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
import matplotlib.colors


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    '''
    https://topex.ucsd.edu/cgi-bin/get_data.cgi
    ----------------  WHOLE MAP ---------------- 
               SF                            LA                    NY                 D/FW
    north    38.06506350841047       34.24              41.08964763912464      
    south    37.14754423773077       33.57              40.46716023330416      
    east    -121.15489648222344      -117.05          -73.08765205412222      
    west    -122.6202709295737      -118.56            -74.38659872661535     
    
    squ
    ---------------- ZOOMED ---------------- 
               SF                  LA                     NY                   D/FW               Rio, Brazil        Boston 
    north  37.88287952770287     34.387631520207854         40.86779016836917         33.274212825511114            -22.77            42.41
    south  37.310850280652446    33.447771783059636      40.58212335516373         32.50505414776004             -23.01            42.32
    east  -121.69064579988729    -116.96541430339893    -73.73082007636665        -96.38849594364828             -43               -70.96
    west  -122.66824837246914    -118.74795634064141     -74.21043817642193       -97.55716661377525              -43.3             -71.18
     
  
    '''
    #downloaded_code()
    modified_code()
    return 

def  modified_code(): 

    plt.rcParams['axes.linewidth'] = 1.
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams.update({'axes.labelsize': 18,
                  'xtick.labelsize': 18,
                  'ytick.labelsize': 18,
                  'axes.titlesize': 32,
                  'font.size': 10})
    
    topography_file        = 'LA_Metropolitan_Zoomed_Area.txt'  
     
    colors_undersea = plt.cm.terrain(np.linspace(0, 0.17, 56))
    colors_land     = plt.cm.terrain(np.linspace(0.25, 1, 200)) 
    
    # combine them and build a new colormap
    colors          = np.vstack((colors_undersea, colors_land))
    cut_terrain_map = matplotlib.colors.LinearSegmentedColormap.from_list('cut_terrain', colors)
    
    
    data = np.loadtxt(topography_file)
    Long = data[:,0]
    Lat  = data[:,1]
    Elev = data[:,2] 
      
    pts    = 100   
    N_lat  = 100
    N_long = 200
    
    R     = 6378.1 * 1E3
    x_dist_max       = (np.max(Lat)-np.min(Lat))*Units.degrees * R # eqn for arc length,  assume earth is a perfect sphere 
    y_dist_max       = (np.max(Long)-np.min(Long))*Units.degrees * R   # eqn for arc length,  assume earth is a perfect sphere 
    
    [long_dist,lat_dist]  = np.meshgrid(np.linspace(0,y_dist_max,N_long),np.linspace(0,x_dist_max,N_lat))
    [long_deg,lat_deg]    = np.meshgrid(np.linspace(np.min(Long),np.max(Long),N_long),np.linspace(np.min(Lat),np.max(Lat),N_lat)) 
    z_deg                 = griddata((Lat,Long), Elev, (lat_deg, long_deg), method='linear')     
         
    norm = FixPointNormalize(sealevel=0,vmax=np.max(z_deg),vmin=np.min(z_deg)) 
    
    fig = plt.figure()
    fig.set_size_inches(8,6)
    axis = fig.add_subplot(1,1,1) 
    
    CS  = axis.contourf(long_deg,lat_deg,z_deg,cmap =cut_terrain_map,norm=norm,levels = 20)  
    cbar = fig.colorbar(CS, ax=axis)     
    cbar.ax.set_ylabel('Elevation above sea level [m]', rotation =  90)  
    axis.set_xlabel('Longitude [°]')
    axis.set_ylabel('Latitude [°]')
     
    #CS   = axis.contourf(long_dist,lat_dist,z_deg,cmap =cut_terrain_map,norm=norm,levels = 20)     
    #cbar = fig.colorbar(CS, ax=axis)        
    #cbar.ax.set_ylabel('Elevation above sea level [m]', rotation =  90) 
    #axis.set_xlabel('x [m]')
    #axis.set_ylabel('y [m]')  
     

     
    return 


def downloaded_code():
    
     
    colors_undersea = plt.cm.terrain(np.linspace(0, 0.17, 56))
    colors_land     = plt.cm.terrain(np.linspace(0.25, 1, 200)) 
    
    # combine them and build a new colormap
    colors          = np.vstack((colors_undersea, colors_land))
    cut_terrain_map = matplotlib.colors.LinearSegmentedColormap.from_list('cut_terrain', colors)
    
    
    data = np.loadtxt('LA_Metropolitan_Area.txt'); 
    Long = data[:,0]
    Lat  = data[:,1]
    Elev = data[:,2] 
    
    tl  =  5 
    tw  =  2 
    S   =  30
    pts = 1000   
     
    [x,y] = np.meshgrid(np.linspace(np.min(Long),np.max(Long),pts),np.linspace(np.min(Lat),np.max(Lat),pts))
    z = griddata((Long, Lat), Elev, (x, y), method='linear')
    x = np.matrix.flatten(x)
    y = np.matrix.flatten(y)
    z = np.matrix.flatten(z)
     
    fig,ax = plt.subplots()
    
    norm = FixPointNormalize(sealevel=0,vmax=np.max(z)-400,vmin=np.min(z)+250)
    
    plt.scatter(x,y,1,z,cmap =cut_terrain_map,norm=norm) 
    cbar = plt.colorbar(label='Elevation above sea level [m]') 
    plt.xlabel('Longitude [°]')
    plt.ylabel('Latitude [°]')
    
    
    plt.gca().set_aspect('equal') 
    
    ax.tick_params(which='major',direction='in',bottom=True, top=True, left=True, right=True,length=tl*2,width=tw+1,color='k')
    ax.minorticks_on()
    ax.tick_params(which='minor',direction='in',bottom=True, top=True, left=True, right=True,color='k',length=tl+1.5,width=tw)
    
    plt.rcParams["font.family"] = "charter"
    plt.rcParams.update({'font.size': S}) 
    
    return 


class FixPointNormalize(matplotlib.colors.Normalize):
    """ 
    Inspired by https://stackoverflow.com/questions/20144529/shifted-colorbar-matplotlib
    Subclassing Normalize to obtain a colormap with a fixpoint 
    somewhere in the middle of the colormap.
    This may be useful for a `terrain` map, to set the "sea level" 
    to a color in the blue/turquise range. 
    """
    def __init__(self, vmin=None, vmax=None, sealevel=0, col_val = 0.21875, clip=False):
        # sealevel is the fix point of the colormap (in data units)
        self.sealevel = sealevel
        # col_val is the color value in the range [0,1] that should represent the sealevel.
        self.col_val = col_val
        matplotlib.colors.Normalize.__init__(self, vmin, vmax, clip)

    def __call__(self, value, clip=None):
        x, y = [self.vmin, self.sealevel, self.vmax], [0, self.col_val, 1]
        return np.ma.masked_array(np.interp(value, x, y)) 
    
if __name__ == '__main__': 
    main()    
    plt.show()
