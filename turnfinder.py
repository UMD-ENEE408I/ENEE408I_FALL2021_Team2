import cv2
import matplotlib.pyplot as plt
import numpy as np

#path 2
#o = [(146, 79), (146, 79), (145, 79), (145, 78), (145, 77), (146, 77), (146, 76), (146, 75), (147, 75), (147, 74), (147, 73), (148, 73), (149, 73), (150, 73), (151, 73), (152, 73), (152, 74), (153, 74), (154, 74), (155, 74), (156, 74), (156, 73), (156, 72), (156, 71), (155, 71), (154, 71), (154, 70), (154, 69), (155, 69), (156, 69), (156, 68), (156, 67), (155, 67), (155, 66), (155, 65), (156, 65), (157, 65), (158, 65), (158, 64), (159, 64), (160, 64), (161, 64), (161, 65), (162, 65), (163, 65), (164, 65), (164, 64), (165, 64), (165, 63), (165, 62), (165, 61), (165, 60), (165, 59), (165, 58), (165, 57), (166, 57), (166, 56), (166, 55), (166, 54), (166, 53), (167, 53), (167, 54), (168, 54), (169, 54), (170, 54), (171, 54), (172, 54), (173, 54), (173, 55), (174, 55), (175, 55), (175, 56), (176, 56), (177, 56), (178, 56), (178, 55), (179, 55), (179, 56), (180, 56), (180, 55), (181, 55), (182, 55), (183, 55), (184, 55), (184, 56), (184, 57), (184, 58), (184, 59), (184, 60), (185, 60), (185, 61), (185, 62), (184, 62), (184, 63), (183, 63), (183, 64), (182, 64), (182, 65), (181, 65), (180, 65), (179, 65), (178, 65), (177, 65), (176, 65), (175, 65), (175, 66), (174, 66), (174, 67), (174, 68), (174, 69), (175, 69), (175, 70), (174, 70), (174, 71), (174, 72), (174, 73), (174, 74), (174, 75), (174, 76), (174, 77), (174, 78), (174, 79), (174, 80), (174, 81), (174, 82), (174, 83), (174, 84), (174, 85), (175, 85), (176, 85), (177, 85), (178, 85), (179, 85), (180, 85), (181, 85), (182, 85), (183, 85), (184, 85), (184, 84), (185, 84), (185, 85), (186, 85), (186, 84), (187, 84), (188, 84), (189, 84), (190, 84), (191, 84), (192, 84), (192, 83), (192, 82), (193, 82), (193, 81), (193, 80), (193, 79), (193, 78), (193, 77), (193, 76), (194, 76), (194, 75), (195, 75), (196, 75), (197, 75), (198, 75), (199, 75), (199, 74), (200, 74), (201, 74), (202, 74), (203, 74), (203, 73), (203, 72), (203, 71), (202, 71), (202, 70), (202, 69), (202, 68), (203, 68), (203, 67), (203, 66), (203, 65), (204, 65), (205, 65), (206, 65), (207, 65), (208, 65), (209, 65), (210, 65), (210, 64), (210, 63), (210, 62), (209, 62), (209, 61), (209, 60), (209, 59), (209, 58), (210, 58), (210, 57), (211, 57), (212, 57), (213, 57), (214, 57), (215, 57), (216, 57), (216, 56), (216, 55), (216, 54), (216, 53), (216, 52), (216, 51), (216, 50), (216, 49), (216, 48), (216, 47), (216, 46), (216, 45), (216, 44), (216, 43), (216, 42), (217, 42), (218, 42), (218, 41), (219, 41), (220, 41), (221, 41), (222, 41), (222, 42), (223, 42), (224, 42), (224, 43), (225, 43)]
#path 3
#o = [(146, 79), (146, 79), (147, 79), (147, 80), (148, 80), (148, 81), (148, 82), (148, 83), (149, 83), (150, 83), (151, 83), (151, 84), (151, 85), (152, 85), (153, 85), (154, 85), (154, 86), (154, 87), (155, 87), (155, 88), (155, 89), (156, 89), (156, 90), (156, 91), (156, 92), (155, 92), (155, 93), (154, 93), (154, 94), (154, 95), (154, 96), (154, 97), (154, 98), (154, 99), (154, 100), (154, 101), (155, 101), (156, 101), (156, 102), (156, 103), (156, 104), (155, 104), (154, 104), (154, 105), (153, 105), (152, 105), (151, 105), (150, 105), (150, 106), (149, 106), (148, 106), (147, 106), (146, 106), (146, 107), (146, 108), (146, 109), (146, 110), (146, 111), (146, 112), (146, 113), (146, 114), (146, 115), (146, 116), (145, 116), (144, 116), (144, 115), (144, 114), (143, 114), (142, 114), (141, 114), (140, 114), (139, 114), (138, 114), (137, 114), (136, 114), (135, 114), (134, 114), (133, 114), (132, 114), (131, 114), (130, 114), (129, 114), (128, 114), (127, 114), (126, 114), (126, 115), (126, 116), (126, 117), (126, 118), (126, 119), (126, 120), (126, 121), (126, 122), (126, 123), (126, 124), (127, 124), (128, 124), (129, 124), (129, 125), (130, 125), (131, 125), (132, 125), (133, 125), (134, 125), (134, 124), (135, 124), (136, 124), (137, 124), (138, 124), (139, 124), (140, 124), (141, 124), (142, 124), (143, 124), (144, 124), (145, 124), (146, 124), (147, 124), (148, 124), (149, 124), (150, 124), (151, 124), (152, 124), (153, 124), (154, 124), (154, 125), (155, 125), (156, 125), (157, 125), (158, 125), (159, 125), (159, 124), (160, 124), (160, 123), (161, 123), (162, 123), (163, 123), (163, 124), (164, 124), (165, 124), (166, 124), (167, 124), (167, 123), (168, 123), (169, 123), (170, 123), (171, 123), (172, 123), (173, 123), (174, 123), (174, 124), (174, 125), (174, 126), (174, 127), (174, 128), (174, 129), (174, 130), (173, 130), (173, 131), (173, 132), (173, 133), (174, 133), (175, 133), (176, 133), (176, 132), (177, 132), (177, 131), (178, 131), (179, 131), (180, 131), (181, 131), (182, 131), (183, 131), (184, 131), (185, 131), (186, 131), (186, 130), (187, 130), (188, 130), (189, 130), (190, 130), (190, 131), (191, 131), (192, 131), (193, 131), (194, 131), (194, 130), (195, 130), (196, 130), (197, 130), (198, 130), (199, 130), (200, 130), (200, 129), (201, 129), (201, 128), (202, 128), (203, 128), (204, 128), (204, 129), (205, 129), (206, 129), (207, 129), (207, 128), (208, 128), (209, 128), (210, 128), (210, 127), (211, 127), (211, 126), (212, 126), (213, 126), (214, 126), (215, 126), (215, 125), (216, 125), (217, 125), (218, 125), (218, 126), (219, 126), (220, 126), (221, 126), (221, 125), (222, 125), (223, 125), (224, 125)]
#p = o[::-1]

#path 5
#o = [(163, 72), (163, 72), (163, 73), (164, 73), (165, 73), (165, 74), (166, 74), (166, 75), (166, 76), (166, 77), (166, 78), (167, 78), (168, 78), (169, 78), (170, 78), (170, 79), (171, 79), (172, 79), (172, 80), (173, 80), (173, 79), (174, 79), (175, 79), (175, 80), (175, 81), (175, 82), (176, 82), (176, 83), (176, 84), (175, 84), (175, 85), (176, 85), (176, 86), (176, 87), (176, 88), (176, 89), (175, 89), (175, 90), (175, 91), (175, 92), (175, 93), (175, 94), (175, 95), (175, 96), (175, 97), (175, 98), (175, 99), (175, 100), (175, 101), (175, 102), (174, 102), (174, 103), (174, 104), (174, 105), (173, 105), (173, 106), (172, 106), (171, 106), (170, 106), (169, 106), (168, 106), (168, 107), (167, 107), (166, 107), (165, 107), (164, 107), (164, 108), (164, 109), (164, 110), (164, 111), (164, 112), (163, 112), (163, 113), (163, 114), (163, 115), (163, 116), (163, 117), (163, 118), (163, 119), (162, 119), (161, 119), (160, 119), (159, 119), (158, 119), (157, 119), (156, 119), (155, 119), (154, 119), (153, 119), (152, 119), (152, 118), (151, 118), (151, 119), (150, 119), (149, 119), (148, 119), (147, 119), (146, 119), (145, 119), (144, 119), (143, 119), (143, 120), (142, 120), (141, 120), (140, 120), (140, 121), (140, 122), (140, 123), (140, 124), (139, 124), (139, 125), (139, 126), (139, 127), (140, 127), (140, 128), (140, 129), (139, 129), (139, 130), (138, 130), (137, 130), (137, 131), (137, 132), (136, 132), (135, 132), (134, 132), (134, 131), (133, 131), (132, 131), (132, 132), (131, 132), (130, 132), (129, 132), (129, 131), (128, 131), (127, 131), (126, 131), (126, 132), (125, 132), (124, 132), (123, 132), (123, 131), (122, 131), (122, 132), (121, 132), (121, 133), (120, 133), (119, 133), (118, 133), (117, 133), (116, 133), (115, 133), (114, 133), (113, 133), (112, 133), (111, 133), (110, 133), (110, 132), (109, 132), (108, 132), (108, 131), (107, 131), (106, 131), (106, 132), (106, 133), (105, 133), (104, 133), (103, 133), (103, 132), (103, 131), (102, 131), (101, 131), (101, 132), (100, 132), (99, 132), (98, 132), (97, 132), (96, 132), (95, 132), (94, 132), (93, 132), (92, 132), (91, 132), (90, 132), (90, 131), (90, 130), (89, 130), (88, 130), (87, 130), (86, 130), (85, 130), (84, 130), (83, 130), (82, 130), (81, 130), (80, 130), (79, 130), (78, 130), (77, 130), (76, 130), (75, 130), (74, 130), (73, 130), (72, 130), (71, 130), (70, 130), (69, 130), (69, 131), (69, 132), (68, 132), (68, 133), (68, 134), (68, 135), (68, 136), (68, 137), (68, 138), (68, 139), (67, 139), (67, 140), (67, 141), (66, 141), (66, 142), (65, 142), (65, 141), (64, 141), (63, 141), (62, 141), (61, 141), (60, 141), (59, 141), (58, 141), (57, 141), (56, 141), (55, 141), (54, 141), (53, 141), (52, 141), (51, 141), (51, 142), (50, 142), (49, 142), (48, 142), (47, 142), (46, 142), (45, 142), (45, 141)]
#path 6
o = o = [(163, 72), (163, 72), (163, 71), (163, 70), (163, 69), (164, 69), (164, 68), (165, 68), (166, 68), (166, 67), (167, 67), (168, 67), (168, 66), (169, 66), (170, 66), (171, 66), (171, 67), (172, 67), (173, 67), (174, 67), (174, 66), (175, 66), (176, 66), (176, 65), (176, 64), (175, 64), (175, 63), (175, 62), (175, 61), (175, 60), (175, 59), (175, 58), (175, 57), (175, 56), (176, 56), (176, 55), (177, 55), (177, 54), (178, 54), (179, 54), (180, 54), (181, 54), (182, 54), (183, 54), (183, 53), (184, 53), (185, 53), (186, 53), (187, 53), (187, 52), (187, 51), (188, 51), (188, 50), (188, 49), (188, 48), (189, 48), (189, 47), (189, 46), (188, 46), (188, 45), (189, 45), (189, 44), (189, 43), (189, 42), (189, 41), (189, 40), (190, 40), (191, 40), (192, 40), (192, 41), (193, 41), (193, 42), (194, 42), (195, 42), (196, 42), (196, 43), (197, 43), (198, 43), (199, 43), (200, 43), (201, 43), (202, 43), (203, 43), (204, 43), (205, 43), (206, 43), (207, 43), (208, 43), (209, 43), (209, 44), (209, 45), (209, 46), (209, 47), (209, 48), (209, 49), (209, 50), (210, 50), (210, 51), (210, 52), (209, 52), (209, 53), (209, 54), (208, 54), (208, 55), (207, 55), (206, 55), (205, 55), (204, 55), (203, 55), (202, 55), (201, 55), (200, 55), (199, 55), (199, 56), (198, 56), (197, 56), (197, 57), (197, 58), (197, 59), (197, 60), (197, 61), (198, 61), (198, 62), (198, 63), (198, 64), (198, 65), (198, 66), (198, 67), (199, 67), (199, 68), (199, 69), (199, 70), (199, 71), (199, 72), (199, 73), (199, 74), (198, 74), (198, 75), (198, 76), (198, 77), (198, 78), (198, 79), (199, 79), (200, 79), (200, 80), (200, 81), (201, 81), (202, 81), (203, 81), (204, 81), (204, 80), (205, 80), (206, 80), (207, 80), (207, 79), (208, 79), (209, 79), (210, 79), (211, 79), (212, 79), (213, 79), (214, 79), (215, 79), (216, 79), (217, 79), (218, 79), (218, 80), (219, 80), (220, 80), (220, 79), (220, 78), (220, 77), (220, 76), (220, 75), (220, 74), (220, 73), (221, 73), (221, 72), (221, 71), (221, 70), (221, 69), (221, 68), (220, 68), (220, 67), (220, 66), (221, 66), (222, 66), (223, 66), (224, 66), (225, 66), (226, 66), (227, 66), (228, 66), (229, 66), (230, 66), (230, 67), (231, 67), (232, 67), (232, 66), (232, 65), (232, 64), (232, 63), (232, 62), (233, 62), (233, 61), (233, 60), (233, 59), (233, 58), (233, 57), (232, 57), (232, 56), (232, 55), (232, 54), (233, 54), (234, 54), (235, 54), (236, 54), (237, 54), (238, 54), (239, 54), (240, 54), (241, 54), (241, 53), (241, 52), (241, 51), (241, 50), (241, 49), (241, 48), (242, 48), (242, 49), (243, 49), (243, 48), (243, 47), (243, 46), (243, 45), (243, 44), (242, 44), (242, 43), (242, 42), (243, 42), (244, 42), (245, 42), (246, 42), (247, 42), (248, 42), (249, 42), (250, 42), (251, 42), (251, 41), (251, 40), (251, 39), (251, 38), (251, 37), (251, 36), (252, 36), (253, 36), (253, 35), (252, 35), (251, 35), (251, 34), (251, 33), (251, 32), (252, 32), (252, 31), (253, 31), (253, 30), (253, 29), (253, 28), (253, 27), (252, 27), (252, 26), (252, 25), (252, 24), (252, 23), (252, 22), (252, 21), (252, 20), (252, 19), (253, 19), (254, 19), (254, 20), (255, 20), (255, 19), (256, 19), (257, 19), (258, 19), (259, 19), (260, 19), (261, 19), (262, 19), (263, 19), (264, 19), (264, 20), (265, 20), (266, 20), (267, 20), (268, 20), (269, 20), (270, 20), (270, 21)]
p = o[::-1]

#print(p)
x = 1
y = 0 
left = 1
right = 0
pos = 1
neg = 0
counter = 0

abs_change = 5


dir = 0
crnt = 3
xy = x
pn = 1
#print(len(o))
#print("blaaaaah")

while(crnt <= len(o)):
    if xy == x:
        pn = p[crnt][0] - p[crnt-3][0]
        if pn < 0:
            pn = neg
        else:
            pn = pos
        #print("here is pn:" + str(pn))
        for i in range(0,500):
            change = p[crnt][1]-p[crnt+i+5][1]
            if abs(change) > abs_change:
                print(change)
                crnt = crnt + i + 5
                #print(i)
                #print(p[crnt])
                if pn == neg:
                    if change < 0:
                        counter += 1
                        print(str(counter)+ ":left")
                    else:
                        counter += 1
                        print(str(counter)+ ":right")
                else:
                    if change < 0:
                        counter += 1
                        print(str(counter)+ ":right")
                    else:
                        counter += 1
                        print(str(counter)+ ":left")
                break
        xy = y
    else:
        pn = p[crnt][1] - p[crnt-3][1]
        if pn < 0:
            pn = neg
        else:
            pn = pos
        for i in range(0,500):
            change = p[crnt][0]-p[crnt+i+5][0]
            if abs(change) > abs_change:
                print(change)
                crnt = crnt + i +5
                #print(p[crnt])
                #print(i)
                if pn == neg:
                    if change < 0:
                        counter += 1
                        print(str(counter)+ ":right")
                    else:
                        counter += 1
                        print(str(counter)+ ":left")
                else:
                    if change < 0:
                        counter += 1
                        print(str(counter)+ ":left")
                    else:
                        counter += 1
                        print(str(counter)+ ":right")
                break
        xy = x






print("this is pn:" + str(pn)) 
print("this is xy:" + str(xy))
print("this is crnt:" + str(crnt))
print("this is coords:" +str(o[crnt]))
