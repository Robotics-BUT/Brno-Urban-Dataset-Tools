from GNSS_plotter import GNSS_plotter
from utils.folders import Folders


def main():
    rec_folders = Folders()
    folder = 'path_to_rec_folders/'
    recs = ['rec_1/',
            'rec_2/',
            'rec_3/']
    folders = [folder+rec+rec_folders.gnss+'pose.txt' for rec in recs];

    output = 'output/path/map.html'
    api_key = 'YOUR_API_KEY'
    plotter = GNSS_plotter(api_key)
    (total_dist, total_time) = plotter.plot_route_list(folders, output, 1000)
    print(' Total distance ' + ' = ', total_dist, ' total time: ', total_time)


if __name__ == '__main__':
    main()
