import bag_builder

def main():
    path_to_rec = 'FOLDER/'
    rec_name = 'REC_NAME/'
    builder = bag_builder.BagBuilder()
    builder.build_bag(path_to_rec, rec_name)

if __name__ == "__main__":
    main()