import numpy as np
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
    north    38.76506350841047                            40.86779016836917 
    south    37.21754423773077                            40.58212335516373 
    east    -121.78489648222344                          -73.67082007636665 
    west    -123.9202709295737                           -74.29043817642193
    
    
    ---------------- ZOOMED ---------------- 
               SF                            LA                     NY                   D/FW
    north  37.88287952770287      34.72620275272516        41.08964763912464        33.274212825511114 
    south  37.310850280652446     33.35923172382056        40.46716023330416        32.50505414776004 
    east  -121.69064579988729     -116.93500827849786     -73.08765205412222       -96.38849594364828
    west  -122.66824837246914    -118.93325253987054      -74.38659872661535      -97.55716661377525
     
  
  
    '''
    
    
    # Combine the lower and upper range of the terrain colormap with a gap in the middle
    # to let the coastline appear more prominently.
    # inspired by https://stackoverflow.com/questions/31051488/combining-two-matplotlib-colormaps
    colors_undersea = plt.cm.terrain(np.linspace(0, 0.17, 56))
    colors_land = plt.cm.terrain(np.linspace(0.25, 1, 200))
    
    
    # combine them and build a new colormap
    colors = np.vstack((colors_undersea, colors_land))
    cut_terrain_map = matplotlib.colors.LinearSegmentedColormap.from_list('cut_terrain', colors)
    
    
    plt.close('all')
    
    data = np.loadtxt('Madagascar_Elevation.txt');
    data
    Long = data[:,0]; Lat = data[:,1]; Elev = data[:,2];
    
    tl = 5;
    tw = 2;
    lw = 3;
    S = 30
    pts=1000000;
     
    [x,y] =np.meshgrid(np.linspace(np.min(Long),np.max(Long),np.sqrt(pts)),np.linspace(np.min(Lat),np.max(Lat),np.sqrt(pts)))
    z = griddata((Long, Lat), Elev, (x, y), method='linear')
    x = np.matrix.flatten(x)
    y = np.matrix.flatten(y)
    z = np.matrix.flatten(z)
    
    cmap = plt.get_cmap('terrain')
    
    
    fig,ax = plt.subplots()
    
    norm = FixPointNormalize(sealevel=0,vmax=np.max(z)-400,vmin=np.min(z)+250)
    
    plt.scatter(x,y,1,z,cmap =cut_terrain_map,norm=norm)
    cbar = plt.colorbar(label='Elevation above sea level [m]')
    cbar.ax.tick_params(size=3,width =1)
    cbar.ax.tick_params(which='major',direction='in',bottom=True, top=True, left=False, right=True,length=tl*2,width=tw+1,color='k')
    cbar.outline.set_linewidth(lw)
    
    plt.xlabel('Longitude [°]')
    plt.ylabel('Latitude [°]')
    
    
    plt.gca().set_aspect('equal')
    plt.xlim(40,52)
    plt.ylim(-10,-28)
    plt.gca().invert_yaxis()
    plt.yticks([-10,-15,-20,-25])
    ax.set_yticks(np.linspace(-28,-10,19), minor=True)
    
    
    ax.tick_params(colors='k')
    ax.spines['top'].set_linewidth(lw)
    ax.spines['bottom'].set_linewidth(lw)
    ax.spines['right'].set_linewidth(lw)
    ax.spines['left'].set_linewidth(lw)
    
    
    ax.tick_params(which='major',direction='in',bottom=True, top=True, left=True, right=True,length=tl*2,width=tw+1,color='k')
    ax.minorticks_on()
    ax.tick_params(which='minor',direction='in',bottom=True, top=True, left=True, right=True,color='k',length=tl+1.5,width=tw)
    
    plt.rcParams["font.family"] = "charter"
    plt.rcParams.update({'font.size': S})
    ax.set_yticks(np.linspace(-28,-10,19), minor=True)
    
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
