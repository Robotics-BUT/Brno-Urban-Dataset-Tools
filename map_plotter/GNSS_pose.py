class GNSS_pose:
    
    def __init__(self, lat, lon, alt, heading, t):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.head = heading
        self.timestamp = t
        
    def is_heading(self):
        return self.head != 0 