def hex2rgb(hexcolor):
    hexcolor = int(hexcolor, base=16) if isinstance(hexcolor, str) else hexcolor
    rgb = ((hexcolor >> 16) & 0xff, (hexcolor >> 8) & 0xff, hexcolor & 0xff)
    return rgb
import cv2

CLASSES=['person','bicycle','car','motorcycle','airplane','bus','train','truck']
hex = ['FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
                '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7']
DATASET_PATH='/home/yihang/avia_road/02/E/image_result/image4/'
# with open('/home/yihang/dev_catkin/src/TrackEval/data/trackers/avia/avia_2d_box_train/ours/data/0000_r.txt','r') as label:
with open('/home/yihang/dev_catkin/src/TrackEval/data/trackers/avia/avia_2d_box_train/ours/data/0004.txt','r') as label:
    data = label.readlines()
    for line in data:
        a = line.split()
        frameid = str(int(a[0]) + 5099)
        w = int(a[4])//2
        img = cv2.imread(DATASET_PATH + frameid + ".jpg")
        pt1 = (round(float(a[6])),round(float(a[7])))
        pt0 = (round((round(float(a[6]) + round(float(a[8])))/2)),round(float(a[7])-5.))
        # pt0 = (int(a[2])+w,int(a[3])-5)
        pt3 = (round(float(a[6])),round(float(a[7])-5.))
        # pt2 = (int(a[2])+int(a[4]),int(a[3])+int(a[5]))
        pt2 = (round(float(a[8])),round(float(a[9])))
        # cv2.putText(img, a[2], pt0, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,0),2)
        cv2.putText(img, a[1], pt3, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255),2)
        # cv2.putText(img, a[1], pt3, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,0),2)
        cv2.rectangle(img, pt1, pt2, (0,0,255), 2)
        cv2.imwrite("/home/yihang/avia_road/02/E/image_result/image4/%s.jpg" %(frameid), img)
        print("writed img: ", frameid)