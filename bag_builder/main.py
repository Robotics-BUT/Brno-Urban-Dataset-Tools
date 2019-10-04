import bag_builder

def main():
    path_to_rec = '/home/autodrive/Data/final/downlink/'
    rec_name = '1_2_7_1'
    builder = bag_builder.BagBuilder()
    builder.build_bag(path_to_rec, rec_name)

if __name__ == "__main__":
    main()