# On Linux, tested with:
# - python			3.11.2
# - PyOpenGL		3.1.6
# - pygame          2.2.0
# - PyGObject       3.42.2

# On Windows:
# - Install PyGi:
#	- See: https://stackoverflow.com/questions/17278953/gstreamer-python-bindings-for-windows
# - Install pyopengl from: https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyopengl
#	- Use: python -m pip install <wheel file>
#	- Note: pyopengl installed via pip doesn't include glut.dll. Remove pip's installed version first

import sys
import numpy as np
from PIL import Image
import time
from math import sin, cos, sqrt, atan2, radians
import signal

import threading

# OpenGL libs
import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

from pymavlink import mavutil

import pygame

class Drone:
	def __init__(self):
		self.x = 0		# North [m]
		self.y = 0		# South [m]
		self.yaw = 0	# Yaw [-180, 180] where 0 points north
		self.alt = 10	# Altitude [m]

class CameraSim:
	def __init__(self, renderer, drone):
		self.renderer = renderer

		self.fps = 7

		self.drone = drone

		self.outWidth = 1280
		self.outHeight = 720

		self.move = False	# For testing

		self.videoHost = "yourhost.com"
		#self.videoHost = "127.0.0.1"
		self.videoPort = 5000

		self.calibrate = False

		self.markerOrientation = 45 # Clockwise

		if renderer == "gst":
			self.initGST()

	def initGST(self):
		# Initialize GStreamer
		Gst.init(None)

		# Create a GStreamer pipeline

		# Local test
		#output = "appsrc name=source ! videoconvert ! autovideosink"

		# Low quality
		output = "appsrc name=source ! videoconvert ! video/x-raw,framerate=" + str(self.fps) + "/1 ! openh264enc ! rtph264pay ! udpsink host=" + str(self.videoHost) + " port=" + str(self.videoPort)

		# High quality
		#output = "appsrc name=source ! videoconvert ! video/x-raw,framerate=" + str(self.fps) + "/1 ! openh264enc bitrate=2000000 ! rtph264pay ! udpsink host=" + str(self.videoHost) + " port=" + str(self.videoPort)

		# Localhost
		#output = "appsrc name=source ! videoconvert ! video/x-raw,framerate=" + str(self.fps) + "/1 ! openh264enc bitrate=2000000 ! rtph264pay ! udpsink host=127.0.0.1 port=5000"

		pipeline = Gst.parse_launch(output)
		pipeline.set_state(Gst.State.PLAYING)

		# Create a GStreamer appsrc element
		self.appsrc = pipeline.get_by_name("source")
		self.appsrc.set_property("caps", Gst.Caps.from_string("video/x-raw,format=RGBA,width=" + str(self.outWidth) + ",height=" + str(self.outHeight) + ",framerate=" + str(self.fps) + "/1"))

	def renderGround(self):
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

		# Transformations are applied in reverse order

		glRotatef(self.drone.yaw * self.factor, 0, 0, 1)
		glTranslatef(self.drone.x, self.drone.y * self.factor, -self.drone.alt)
		glRotatef(self.markerOrientation * self.factor, 0, 0, 1) # rotate the marker through its center
		glTranslatef(-self.markerWidth / 2.0, -self.markerHeight / 2.0, 0) # move marker to its center

		if self.renderer == "pygame":
			glRotatef(180, 0, 0, 1) # pygame is rotated

		# Center in (0,0)

		glBegin(GL_QUADS)
		glTexCoord2f(0, 0); glVertex3f(0, 0, 0)
		glTexCoord2f(1, 0); glVertex3f(self.markerWidth, 0, 0)
		glTexCoord2f(1, 1); glVertex3f(self.markerWidth, self.markerHeight, 0)
		glTexCoord2f(0, 1); glVertex3f(0, self.markerHeight, 0)
		glEnd()

	def initLighting(self):
		# Set up the lighting
		glEnable(GL_LIGHTING)
		glEnable(GL_LIGHT0)
		glLightfv(GL_LIGHT0, GL_POSITION, (-0, 0, 10, 1.0))
		glLightfv(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1.0))
		glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.5, 0.5, 0.5, 1.0))

	def initGLUT(self):
		glutInit()
		#glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE)
		#glutInitWindowSize(self.outWidth, self.outHeight)
		glutInitWindowSize(50, 10) # A window is required even when streaming using "gst". TODO: Find a way to avoid using X when streaming.
		glutCreateWindow("")
		glutHideWindow() # TODO: Doesn't hide
		#glutDisplayFunc(self.renderGLUT)

		glClearColor(0, 0, 0, 0)

	def initOffScreen(self):
		fbWidth = self.outWidth
		fbHeight = self.outHeight

		# Setup framebuffer
		framebuffer = glGenFramebuffers (1)
		glBindFramebuffer(GL_FRAMEBUFFER, framebuffer)

		# Setup colorbuffer
		colorbuffer = glGenRenderbuffers (1)
		glBindRenderbuffer(GL_RENDERBUFFER, colorbuffer)
		glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, fbWidth, fbHeight)
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorbuffer) 

		# Setup depthbuffer
		depthbuffer = glGenRenderbuffers (1)
		glBindRenderbuffer(GL_RENDERBUFFER,depthbuffer)
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, fbWidth, fbHeight)
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthbuffer)

		# check status
		status = glCheckFramebufferStatus(GL_FRAMEBUFFER)
		if status != GL_FRAMEBUFFER_COMPLETE:
			print( "Failed to init frame buffer")
			exit(1)

		# Set up viewport
		glViewport(0, 0, fbWidth, fbHeight)

	def initGL(self):
		if self.renderer == "pygame":
			# Initialize pygame and OpenGL
			pygame.init()
			display = (1280 / 2, 720 / 2)
			pygame.display.set_mode(display, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)

			# pygame has its axis inverted => use glOrtho() to make consistent with GLUT
			glMatrixMode(GL_PROJECTION)
			glLoadIdentity()
			glOrtho(0, 1280, 720, 0, 0.1, 0)
			self.factor = 1

		elif self.renderer == "gst":
			self.initGLUT()
			self.initOffScreen()
 			self.factor = -1

		self.windowResized(self.outWidth, self.outHeight)

		image = Image.open(self.image_path)
		image_data = image.tobytes()

		self.imgPixelWidth, self.imgPixelHeight = image.size

		self.markerWidth = self.imgPixelWidth / self.imgPixelsPerMeter
		self.markerHeight = self.imgPixelHeight / self.imgPixelsPerMeter

		glEnable(GL_DEPTH_TEST)

		self.initLighting()

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1)

		# Set up the ground plane
		glEnable(GL_TEXTURE_2D)
		texture_id = glGenTextures(1)
		glBindTexture(GL_TEXTURE_2D, texture_id)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.imgPixelWidth, self.imgPixelHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data)
		glEnable(GL_TEXTURE_GEN_S)
		glEnable(GL_TEXTURE_GEN_T)
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR)
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR)
		glTexGenfv(GL_S, GL_OBJECT_PLANE, (1 / self.markerWidth, 0, 0, 0))
		glTexGenfv(GL_T, GL_OBJECT_PLANE, (0, 1 / self.markerHeight, 0, 0))

	def windowResized(self, w, h):
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		#gluPerspective(70, (float)(w) / h, 0.1, 50.0)
		# DJI Mini SE calibration parameters: fx: 1009.570496, fy: 1011.441406, fovx: 64.743914, fovy: 39.184104, w: 1280, h: 720
		gluPerspective(39.184104, (float)(w) / h, 0.1, 50.0)

	def doMove(self):
		if self.move:
			self.drone.yaw += 1
			self.drone.alt -= 0.05
			if self.drone.alt < 0:
				self.drone.alt = 10

	def run(self):
		self.initGL()

		if self.renderer == "pygame":
			self.runPyGame()

		elif self.renderer == "gst":
			self.running = True
			try:
				while self.running:
					start_time = time.time()
					camSim.renderGLUT()
					camSim.sendFrame()
					elapsed_time = time.time() - start_time
					time.sleep(max(1.0 / camSim.fps - elapsed_time, 0))

					self.doMove()

			except:
				raise
				return

	def runPyGame(self):
		while True:
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()
					quit()
				elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
					pygame.quit()
					quit()
				elif event.type == pygame.VIDEORESIZE:
					self.windowResized(event.w, event.h)
			
			self.renderGround()

			if self.renderer == "pygame":
				pygame.display.flip()
				pygame.time.wait(10)

	def renderGLUT(self):
		self.renderGround()
		glFlush()

		glutSwapBuffers()

	def sendFrame(self):
		frame = glReadPixels(0, 0, self.outWidth, self.outHeight, GL_RGBA, GL_UNSIGNED_BYTE)
		buffer = Gst.Buffer.new_allocate(None, len(frame), None)
		buffer.fill(0, frame)
		camSim.appsrc.emit("push-buffer", buffer)

class MavlinkThread():
	def __init__(self, drone):
		self.markerPos = None
		self.drone = drone
	
	def start(self):
		self.thread = threading.Thread(target=self.run)
		self.thread.start()
	
	def stop(self):
		self.running = False
		self.thread.interrupt()

	# The camera jumps a little durnig the last inches because the MAVLink location is provided in Lat/Lng with a precision of 7 digits.
	# This functions does some esthetical smoothing.
	def smoothLatLng(self, newVal, oldVal):
		e = abs(newVal - oldVal)
		if e <= 1e7:
			f = 0.9
			return (1 - f) * newVal + f * oldVal
		else:
			return newVal

	def run(self):
		print("MAVLink thread started")

		master = mavutil.mavlink_connection('udp:0.0.0.0:14551')

		self.running = True
		while self.running:
			try:
				msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

				lat = msg.lat / 1e7
				lon = msg.lon / 1e7
				alt = msg.alt / 100.0
				yaw = msg.hdg / 100.0 - 360 # yaw [-180, 180]

				# Set marker pos to initial pos
				if self.markerPos is None:
					print("Drone connected")
					offsetX = 0.000001
					offsetY = 0.000001
					self.markerPos = (lat + offsetX, lon + offsetY)

				delta_lat_m, delta_lng_m = delta_lat_lng_to_meters(lat, lon, self.markerPos[0], self.markerPos[1])

				self.drone.x = self.smoothLatLng(self.drone.x, delta_lng_m * 1e7)
				self.drone.y = self.smoothLatLng(self.drone.y, delta_lat_m * 1e7)

				self.drone.alt = alt / 10 # TEMP: / 10
				self.drone.yaw = yaw

			except mavutil.MavError as e:
				print('Error: ', e)
				return

def delta_lat_lng_to_meters(lat1, lng1, lat2, lng2):
	"""
	Convert delta latitude and longitude to meters.

	Parameters:
	- lat1: float, latitude of the first point in degrees
	- lng1: float, longitude of the first point in degrees
	- lat2: float, latitude of the second point in degrees
	- lng2: float, longitude of the second point in degrees

	Returns:
	- tuple of two floats, delta in meters for the latitude and longitude directions
	"""

	# approximate radius of earth in meters
	R = 6371000

	# convert decimal degrees to radians
	lat1_rad = radians(lat1)
	lng1_rad = radians(lng1)
	lat2_rad = radians(lat2)
	lng2_rad = radians(lng2)

	# calculate the differences in radians
	delta_lat_rad = lat2_rad - lat1_rad
	delta_lng_rad = lng2_rad - lng1_rad

	# calculate the delta in meters using the haversine formula
	a = sin(delta_lat_rad / 2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lng_rad / 2)**2
	c = 2 * atan2(sqrt(a), sqrt(1 - a))
	delta_lat_m = R * c * (1 / cos(lat1_rad)) * delta_lat_rad
	delta_lng_m = R * c * delta_lng_rad

	return delta_lat_m, delta_lng_m

def signal_handler(signal, frame):
	print("Closing...")

	camSim.running = False

	mavlinkThread.stop()

	glutLeaveMainLoop()

	exit(0)

#signal.signal(signal.CTRL_C_EVENT, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

drone = Drone()
mavlinkThread = MavlinkThread(drone)
mavlinkThread.start()

camSim = CameraSim("gst", drone)		# Stream
#camSim = CameraSim("pygame", drone)	# Test

camSim.image_path = "images/landing-target.jpg"
camSim.imgPixelsPerMeter = 466 / 0.47	# 466 [px] = 0.47 [m]
#camSim.calibrate = True	# Not implemented. TODO: Export random images to calibrate the simulated camera. See instructions here: https://github.com/kripper/vision-landing-2/
#camSim.move = True
camSim.run()
