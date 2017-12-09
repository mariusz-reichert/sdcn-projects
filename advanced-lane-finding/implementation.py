import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimage
import glob
from moviepy.editor import VideoFileClip
import sys
from functools import partial
from os.path import basename

PROJECT_ROOT = './CarND-Advanced-Lane-Lines/'
CALIBRATION_PATH = PROJECT_ROOT+'camera_cal/'

WIDTH = 1280
HEIGHT = 720

# Define conversions in x and y from pixels space to meters
YM_PER_PX = 30/HEIGHT # meters per pixel in y dimension
XM_PER_PX = 3.7/700 # meters per pixel in x dimension


# number of consecutive bad frames (lines not detected correctly) after which
# lanes searching process starts from scratch
CBF = 3


# coordinates of points representing rectangle, found by visually inspecting
# test image straight_lines1.jpg
SRC = np.float32([[200,678], [594,450], [714,450], [1108,678]])
DST = np.float32([[[SRC[0][0],HEIGHT], [SRC[0][0],0], \
                   [SRC[-1][0],0], [SRC[-1][0],HEIGHT]]])

PLOTY = np.linspace(0, HEIGHT-1, num=HEIGHT)

# controls the rate with which new lanes replaces old ones, this is for smoothing
ALPHA = 0.2

EMPTY_COEFF = np.array([0,0,0], dtype='float')


class Line():
    def __init__(self):
        self.coeff = np.array([0,0,0], dtype='float') 
        self.prev = None

    def add(self, fit):
        if not np.array_equal(self.coeff, EMPTY_COEFF):
            new_coeff = (1.0-ALPHA)*self.coeff + ALPHA*fit
        else:
            new_coeff = fit

        self.prev = self.coeff
        self.coeff = new_coeff

    def revoke(self):
        if self.prev != None:
            self.coeff = self.prev

L_LINE = Line()
R_LINE = Line()


def calibrate_camera(nx, ny, verbose=False, extension='jpg'):
    objp = np.zeros((ny*nx,3), np.float32)
    objp[:,:2] = np.mgrid[0:nx, 0:ny].T.reshape(-1,2)
    objpoints = [] # 3d points in real world space
    imgpoints = [] # 2d points in image plane
    img_size = None
    fnames = glob.glob(CALIBRATION_PATH+'*.'+extension)

    for idx, fname in enumerate(fnames):
        img = mpimage.imread(fname)
        ret, corners = cv2.findChessboardCorners(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), (nx,ny), None)

        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            if verbose:
                cv2.drawChessboardCorners(img, (nx,ny), corners, ret)
                cv2.imwrite(CALIBRATION_PATH+'result_'+str(idx)+'.jpg', img)

        if not img_size:
            img_size = (img.shape[1], img.shape[0])

    # return calibration data: ret, mtx, dist, rvecs, tvecs
    return cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)


def warp(img, src, dst):
    # calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # warp undistorted img
    return cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))


def get_sobel(gray_img, direction, kernel=3, thresholds=(0,255)):
    if direction == 'x':
        s = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0, ksize = kernel)
    elif direction == 'y':
        s = cv2.Sobel(gray_img, cv2.CV_64F, 0, 1, ksize = kernel)
    else:
        raise ValueError("Not supported direction " + direction + " given.")

    abs_s = np.absolute(s)
    scaled_s = np.uint8(255*abs_s/np.max(abs_s))
    binary_img = np.zeros_like(scaled_s)
    binary_img[(scaled_s >= thresholds[0]) & (scaled_s <= thresholds[1])] = 1
    return binary_img


def mag_thresh(gray_img, kernel=3, thresh=(0,255)):
    sx = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0, ksize = kernel)
    sy = cv2.Sobel(gray_img, cv2.CV_64F, 0, 1, ksize = kernel)

    gradmag = np.sqrt(sx**2 + sy**2)
    gradmag = (gradmag/((np.max(gradmag)/255))).astype(np.uint8) 

    binary_img = np.zeros_like(gradmag)
    binary_img[(gradmag >= thresh[0]) & (gradmag <= thresh[1])] = 1
    return binary_img


def dir_thresh(gray_img, kernel=3, thresh=(0, np.pi/2)):
    sx = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0, ksize=kernel)
    sy = cv2.Sobel(gray_img, cv2.CV_64F, 0, 1, ksize=kernel)

    # Take the absolute value of the gradient direction, 
    absgraddir = np.arctan2(np.absolute(sy), np.absolute(sx))
    binary_img =  np.zeros_like(absgraddir)
    binary_img[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1
    return binary_img


# function that thresholds the S-channel of HLS
def thresh_s_channel(img, thresh=(0, 255)):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    s_channel = hls[:,:,2]
    binary_output = np.zeros_like(s_channel)
    binary_output[(s_channel > thresh[0]) & (s_channel <= thresh[1])] = 1
    return binary_output


def draw_text(img, lines, base = 0):
    for idx, line in enumerate(lines):
        cv2.putText(img, line, (10, base + 20*(idx+1)), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,0), 1)


def save_search_vizualization(img, L_fit, R_fit):
    L_fitx, R_fitx = generate_fitx_values(L_fit, R_fit)
    plt.figure()
    plt.imshow(img)
    plt.plot(L_fitx, PLOTY, color='yellow')
    plt.plot(R_fitx, PLOTY, color='yellow')
    plt.xlim(0, WIDTH)
    plt.ylim(HEIGHT, 0)


def extract_lines_pixel_positions(nonzerox, nonzeroy, L_lane_inds, R_lane_inds):
    Lx = nonzerox[L_lane_inds]
    Ly = nonzeroy[L_lane_inds] 
    Rx = nonzerox[R_lane_inds]
    Ry = nonzeroy[R_lane_inds]
    return (Lx, Ly, Rx, Ry)


def fit_2nd_order_poly(Lx, Ly, Rx, Ry):
    L_fit = np.polyfit(Ly, Lx, 2)
    R_fit = np.polyfit(Ry, Rx, 2)
    return L_fit, R_fit


def generate_fitx_values(L_fit, R_fit):
    L_fitx = L_fit[0]*PLOTY**2 + L_fit[1]*PLOTY + L_fit[2]
    R_fitx = R_fit[0]*PLOTY**2 + R_fit[1]*PLOTY + R_fit[2]
    return L_fitx, R_fitx


def project_lines(undist):
    L_fitx, R_fitx = generate_fitx_values(L_LINE.coeff, R_LINE.coeff)

    # Create an image to draw the lines on
    color_warp = np.zeros_like(undist).astype(np.uint8)

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_L = np.array([np.transpose(np.vstack([L_fitx, PLOTY]))])
    pts_R = np.array([np.flipud(np.transpose(np.vstack([R_fitx, PLOTY])))])
    pts = np.hstack((pts_L, pts_R))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

    unwarped = warp(color_warp, DST, SRC)
    # Combine the result with the original image
    result = cv2.addWeighted(undist, 1, unwarped, 0.3, 0)

    return result


# takes binary warped img and finds lines in it. Based on Udacity code
def find_lines_from_scratch(img, verbose=False):
    # Take a histogram of the bottom half of the image
    hist = np.sum(img[img.shape[0]/2:,:], axis=0)
    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((img, img, img))*255
    # Find the peak of the left and right halves of the hist
    # These will be the starting point for the left and right lines
    midpoint = np.int(hist.shape[0]/2)
    Lx_base = np.argmax(hist[:midpoint])
    Rx_base = np.argmax(hist[midpoint:]) + midpoint

    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(img.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    Lx_curr = Lx_base
    Rx_curr = Rx_base
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 100
    # Create empty lists to receive left and right lane pixel indices
    L_lane_inds = []
    R_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        win_x_L_low = Lx_curr - margin
        win_x_L_high = Lx_curr + margin
        win_x_R_low = Rx_curr - margin
        win_x_R_high = Rx_curr + margin

        # Draw the windows on the visualization image
        cv2.rectangle(out_img,(win_x_L_low,win_y_low),(win_x_L_high,win_y_high),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_x_R_low,win_y_low),(win_x_R_high,win_y_high),(0,255,0), 2) 
        # Identify the nonzero pixels in x and y within the window
        good_L_inds = ((nonzeroy >= win_y_low) & \
                       (nonzeroy < win_y_high) & \
                       (nonzerox >= win_x_L_low) & \
                       (nonzerox < win_x_L_high)).nonzero()[0]
        good_R_inds = ((nonzeroy >= win_y_low) & \
                       (nonzeroy < win_y_high) & \
                       (nonzerox >= win_x_R_low) & \
                       (nonzerox < win_x_R_high)).nonzero()[0]
        # Append these indices to the lists
        L_lane_inds.append(good_L_inds)
        R_lane_inds.append(good_R_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_L_inds) > minpix:
            Lx_curr = np.int(np.mean(nonzerox[good_L_inds]))
        if len(good_R_inds) > minpix:        
            Rx_curr = np.int(np.mean(nonzerox[good_R_inds]))

    L_lane_inds = np.concatenate(L_lane_inds)
    R_lane_inds = np.concatenate(R_lane_inds)

    (Lx, Ly, Rx, Ry) = extract_lines_pixel_positions(nonzerox, nonzeroy, L_lane_inds, R_lane_inds)
    L_fit, R_fit = fit_2nd_order_poly(Lx, Ly, Rx, Ry)


    if verbose:
        plt.figure()
        plt.plot(hist)


        out_img[nonzeroy[L_lane_inds], nonzerox[L_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[R_lane_inds], nonzerox[R_lane_inds]] = [0, 0, 255]
        save_search_vizualization(out_img, L_fit, R_fit)

    return L_fit, R_fit

# takes binary warped img and finds lines in it. Based on Udacity code
def find_lines_with_hint(img, verbose=False):
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    L_lane_inds = ((nonzerox > (L_LINE.coeff[0]*(nonzeroy**2) + L_LINE.coeff[1]*nonzeroy + L_LINE.coeff[2] - margin)) & (nonzerox < (L_LINE.coeff[0]*(nonzeroy**2) + L_LINE.coeff[1]*nonzeroy + L_LINE.coeff[2] + margin))) 
    R_lane_inds = ((nonzerox > (R_LINE.coeff[0]*(nonzeroy**2) + R_LINE.coeff[1]*nonzeroy + R_LINE.coeff[2] - margin)) & (nonzerox < (R_LINE.coeff[0]*(nonzeroy**2) + R_LINE.coeff[1]*nonzeroy + R_LINE.coeff[2] + margin)))  

    (Lx, Ly, Rx, Ry) = extract_lines_pixel_positions(nonzerox, nonzeroy, L_lane_inds, R_lane_inds)
    L_fit, R_fit = fit_2nd_order_poly(Lx, Ly, Rx, Ry)

    if verbose != None:
        # Create an image to draw on and an image to show the selection window
        out_img = np.dstack((img, img, img))*255
        window_img = np.zeros_like(out_img)
        # Color in left and right line pixels
        out_img[nonzeroy[L_lane_inds], nonzerox[L_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[R_lane_inds], nonzerox[R_lane_inds]] = [0, 0, 255]

        L_fitx, R_fitx = generate_fitx_values(L_LINE.coeff, R_LINE.coeff)

        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        L_line_window1 = np.array([np.transpose(np.vstack([L_fitx-margin, PLOTY]))])
        L_line_window2 = np.array([np.flipud(np.transpose(np.vstack([L_fitx+margin, PLOTY])))])
        L_line_pts = np.hstack((L_line_window1, L_line_window2))
        R_line_window1 = np.array([np.transpose(np.vstack([R_fitx-margin, PLOTY]))])
        R_line_window2 = np.array([np.flipud(np.transpose(np.vstack([R_fitx+margin, PLOTY])))])
        R_line_pts = np.hstack((R_line_window1, R_line_window2))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(window_img, np.int_([L_line_pts]), (0,255, 0))
        cv2.fillPoly(window_img, np.int_([R_line_pts]), (0,255, 0))
        out_img = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

        save_search_vizualization(out_img, L_fit, R_fit)

    return L_fit, R_fit

def recalc_radius():
    if L_LINE.coeff == None:
        return 0

    fitx = L_LINE.coeff[0]*PLOTY**2 + L_LINE.coeff[1]*PLOTY + L_LINE.coeff[2]
    y_eval = np.max(PLOTY)

    fit_cr = np.polyfit(PLOTY*YM_PER_PX, fitx*XM_PER_PX, 2)
    radius = ((1 + (2*fit_cr[0]*y_eval*YM_PER_PX + fit_cr[1])**2)**1.5) / np.absolute(2*fit_cr[0])

    return radius


def recalc_offset():
    L_fitx, R_fitx = generate_fitx_values(L_LINE.coeff, R_LINE.coeff)

    line_center = L_fitx[HEIGHT-1] + (R_fitx[HEIGHT-1] - L_fitx[HEIGHT-1]) / 2
    return abs(WIDTH/2 - line_center) * XM_PER_PX


def filter_out_bad_frames():
    global CBF
    if (L_LINE.coeff[0] > 0.0003 and R_LINE.coeff[0] < 0.0003) or R_LINE.coeff[0] > 0.0006:
        CBF = CBF + 1
        L_LINE.revoke()
        R_LINE.revoke()
    else:
        CBF = 0


def search(img, verbose=False):
    if CBF >= 3:
        search_func = find_lines_from_scratch
    else:
        search_func = find_lines_with_hint

    return search_func(img, verbose)


def process_image(mtx, dist, img):
    global CBF

    # pipeline:
    undistorted = cv2.undistort(img, mtx, dist, None, mtx)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_RGB2GRAY)
    gradx = get_sobel(gray, 'x', 9, (10,100))
    s_binary = thresh_s_channel(undistorted, (75,255))

    combined = np.zeros_like(gradx).astype(np.uint8) 
    combined[(gradx == 1) | (s_binary == 1)] = 1
    
    warped= warp(combined, SRC, DST)

    L_fit, R_fit = search(warped)

    L_LINE.add(L_fit)
    R_LINE.add(R_fit)

    filter_out_bad_frames()

    draw_text(undistorted, \
              ["Radius  : {0:.0f}m".format(recalc_radius()), \
               "Offset  : {0:.2f}m".format(recalc_offset())])

    return project_lines(undistorted)


def main(video_file, output_file_name):

    _, mtx, dist, _, _ = calibrate_camera(nx=9, ny=6)

    parameterized_processing_pipeline = partial(process_image, mtx, dist)

    video = VideoFileClip(video_file)
    processed_video = video.fl_image(parameterized_processing_pipeline)
    processed_video.write_videofile(output_file_name, audio=False)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])

