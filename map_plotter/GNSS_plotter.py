import gmplot
import csv
from tqdm import tqdm
from geopy.distance import vincenty

from GNSS_pose import GNSS_pose

class GNSS_plotter:
    
    def __init__(self, api_key):
        self.api_key = api_key
    
    
    def plot_route(self, input_folder, output_map, points_no = 1000):
        pose_list = self._read_gnss_poses(input_folder + 'gnss/pose.txt')
        lat, lon, lat_h, lon_h = self._split_by_heading(pose_list)
        self._plot_map(lat, lon, lat_h, lon_h, output_map, points_no)
        
    def plot_route_list(self, input_files, output_map, points_no = 1000):
        lat = list()
        lon = list()
        lat_h = list()
        lon_h = list()
        tot_dist = 0
        tot_time = 0
        for f in input_files:
            
            distance = self.get_total_lenght(f)
            time = self._get_total_time(f)

            print('File: ' + f + ' distance: ' + str(distance) + ' time: ' + str(time))
            pose_list = self._read_gnss_poses(f)
            la, lo, la_h, lo_h = self._split_by_heading(pose_list)
            lat += la
            lon += lo
            lat_h += la_h
            lon_h += lo_h
            
            tot_dist += distance
            tot_time += time
            
        self._plot_map(lat, lon, lat_h, lon_h, output_map, points_no)
        
        return (tot_dist, tot_time)
        
    def get_total_lenght(self, f):
        
        total_dist = 0
        dist = 0
        pose_list = self._read_gnss_poses(f)
        for i,coord in enumerate(pose_list):
            if i == 0:
                continue
            dist += self._distance_between_wgs_coords(pose_list[i-1], pose_list[i])
        total_dist += dist
        
        return total_dist
        
    def _get_total_time(self, f):

        with open(f) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            times = list()
            for row in tqdm(csv_reader):
                times.append(int(row[0]))
        return (times[-1] - times[0])/1e9           
            
        
    def _read_gnss_poses(self, path):
        coord_list = list()
        with open(path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in tqdm(csv_reader):
                if (len(row) < 5):
                    continue

                try:
                    time = int(row[0])
                    lat = float(row[1])
                    lon = float(row[2])
                    alt = float(row[3])
                    head = float(row[4])
                    coord_list.append( GNSS_pose(lat, lon, alt, head, time ) )
                except:
                    print("Warning: unable to convert ", row )
                
        return coord_list
    
    
    def _split_by_heading(self, coord_list):
        lat_list = list()
        lon_list = list()
        lat_list_h = list()
        lon_list_h = list()
        for gnss_pose in coord_list:
            if gnss_pose.is_heading():
                lat_list_h.append(gnss_pose.lat)
                lon_list_h.append(gnss_pose.lon)
            else:
                lat_list.append(gnss_pose.lat)
                lon_list.append(gnss_pose.lon)
        return lat_list, lon_list, lat_list_h, lon_list_h
    

    def _plot_map(self, lat, lon, lat_h, lon_h, output_file, pn):
        size = 3
        step = int((len(lat_h)+len(lat)) / pn)
        gmap=gmplot.GoogleMapPlotter(lat[0], lon[0], 18)
        gmap.apikey = self.api_key
        gmap.scatter(lat[0::step], lon[0::step], '#A03030', size=size, marker=False)
        gmap.scatter(lat_h[0::step], lon_h[0::step], '#30A030', size=size, marker=False)
        gmap.draw(output_file)
        
    def _distance_between_wgs_coords(self, pose_1, pose_2):

        start = (pose_1.lat, pose_1.lon)
        stop = (pose_2.lat, pose_2.lon)
        return vincenty(start, stop).meters
