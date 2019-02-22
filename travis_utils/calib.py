import cv2
import numpy as np
import ArucoTaura as AT
import cv2.aruco as aruco
import glob
import os

# Author: Eduardo Cattani
# TauraBots Team

class calib:
	def __init__(self,arucolen,path,dic_aruco=cv2.aruco.DICT_6X6_250):
		
		self.cap = cv2.VideoCapture(path)
		self.arucos = AT.ArucoTaura(dic_aruco, 0.2)
		step = arucolen/2
		self.pattern_points = (np.array([[-step,step,0.0],[step,step,0.0],[step,-step,0.0],[-step,-step,0.0]], dtype=np.float32)).reshape(1,-1,3)
		self.lista_corners = []
		self.target = 60
		self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
		self.board = cv2.aruco.CharucoBoard_create(5, 7, 0.04, 0.03,self.dict)
		self.dic_aruco = dic_aruco
		print "#Starting the calibration"
		reutilizar = int(input("Reutilizar as fotos? [1/0]"))
		if reutilizar == 1: self.load = True
		else: self.load = False

	def process(self):
		i = 0
		os.system("rm -rf fotos")
		if self.load:	
			os.system("cp -r fotos2 fotos")
		else:
			os.system("mkdir fotos")
			ret = True
			while(ret==True):
				i+=1
				_, imc = self.cap.read()
				try:
					imc.shape
					cv2.imwrite("fotos/"+str(i)+".jpg",imc)
				except:
					ret = False
				

			lista_imagens = glob.glob("fotos/*jpg")
			qnt_img = len(lista_imagens)
			print "Total of photos: \n", qnt_img
			for path in lista_imagens:				
				
				frame = cv2.imread(path,0)

				self.arucos.feed(frame)	
				corners = self.arucos.findCorners(2)
				markers = cv2.aruco.detectMarkers(frame,self.dict)
				if len(markers[0])>14:
				
					pass
					
				else:
					os.system("rm "+path)
			os.system("cp -r fotos fotos2")

		lista_imagens = glob.glob("fotos/*jpg")
		qnt_img = len(lista_imagens)
		print "Total of photos: \n", qnt_img
		lista_var = []
		lista_var = [[cv2.Laplacian(cv2.imread(path, 0), cv2.CV_64F).var(), path] for path in lista_imagens]
		num = len(lista_var)*0.3
		print num, num.__class__
		ordenada = sorted(lista_var, key = lambda x:x[0])
		lista_remove = ordenada[:int(num)]
		#for parte in lista_remove:
		#	os.system("rm "+ str(parte[1]))
		print "Minimum variation = " + str(lista_remove[-1][0])
		
		lista_imagens = glob.glob("fotos/*jpg")
		frame = cv2.imread(lista_imagens[0],0)
		self.x, self.y= frame.shape

		n = len(lista_imagens)

		part = int(n/self.target)
		rest = int(n%self.target)

		ordenada = ordenada[int(num):]
		lista_remove = ordenada[:int(rest)]
		for parte in lista_remove:
			os.system("rm "+ str(parte[1]))
		delete = []

		for grupo in range(self.target):
			lista = []
			
			for candidato in range(part):
				index = grupo*part+candidato
				print grupo, candidato, index
				path = lista_imagens[index]
				img = cv2.imread(path)
				try:
					img.shape
				except:
					break
				
				n = len(cv2.aruco.detectMarkers(img,self.dict))
				lista.append([path, n])
			exclude = sorted(lista, key = lambda x:x[1], reverse = True)[:-1]
			for excluir in exclude:
				delete.append(excluir)
		
		for parte in delete:
			path = parte[0]
			os.system("rm "+path)


		lista_imagens = glob.glob("fotos/*jpg")
		
		dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
		board = cv2.aruco.CharucoBoard_create(5, 7, 0.04, 0.03,dictionary)
		img = board.draw((200*3,200*3))

		#Dump the calibration board to a file
		cv2.imwrite('charuco.png',img)
		lista_imagens = glob.glob("fotos/*jpg")

		#Start capturing images for calibration


		allCorners = []
		allIds = []
		decimator = 0
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

		for path in lista_imagens:

			frame = cv2.imread(path)
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			res = cv2.aruco.detectMarkers(gray,self.dict)

			if len(res[0])>0:
				res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,self.board)
				if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
					allCorners.append(cv2.cornerSubPix(gray,res2[1],(11,11),(-1,-1),criteria))
					allIds.append(res2[2])

				cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

			cv2.imshow('frame',gray)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			decimator+=1

			imsize = gray.shape



		retval, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
		"""
		objp = np.zeros((6*4,3), np.float32)
		objp[:,:2] = np.mgrid[0:6,0:4].T.reshape(-1,2)		
		objpoints = objp
		tot_error = 0
		for i in xrange(len(objpoints)):
			imgpoints2, _ = cv2.projectPoints(objpoints, rvecs[i], tvecs[i], mtx, dist)
			if allCorners[i].dtype == imgpoints2.dtype:
				error = cv2.norm(allCorners[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
				tot_error += error

		print "total error: ", mean_error/len(objpoints)
		"""
		print "Done"
		print [mtx, dist]	

				
		self.teste([mtx,dist])
		return [new_camera_matrix, cal[2]]

		


	def teste(self, vector):

		self.arucos = AT.ArucoTaura(aruco.DICT_6X6_250, 0.2, vector)
		
		img = np.zeros([self.x,self.y,3],np.uint8)
		

		pontos = [[50,50],[450,50],[450,450], [50,450]]

		lista_imagens = glob.glob("fotos/*jpg")
		frame = cv2.imread(lista_imagens[2],0)

		self.x, self.y = frame.shape
		self.arucos.feed(frame)
		self.arucos.drawAruco(True)
		
		pos = self.arucos.findPos(2)	
		corners = self.arucos.findCorners(2)

		pontos_3d = [[x[0],x[1],0] for x in pontos]
		
		
		
		lista_points = []
		
		for ponto in self.pattern_points.reshape(4,3):
			
			
			x,y,z = ponto

			(point, jacobian) = cv2.projectPoints(np.array([(float(x/2), float(y/2) , float(z/2))]),pos[0],pos[1],vector[0], vector[1])
			point = (int(point[0][0][0]), int(point[0][0][1]))
			print point
			cv2.circle(img, (point[0],point[1]), 20, (0,0,255), -1)
			lista_points.append(point)
					
		
		
		pts2 = np.float32([[0,0],[500,0],[500,500],[0,500]])
		pts1  = corners.reshape(4,2)
		M = cv2.getPerspectiveTransform(pts1,pts2)
		img = cv2.warpPerspective(img,M,(500,500))	

		pontos = [[125,125],[375,125],[375,375], [125,375]]

		for ponto in pontos:
			cv2.line(img, (ponto[0]-20,ponto[1]), (ponto[0]+20,ponto[1]), (255,255,255))
			cv2.line(img, (ponto[0],ponto[1]-20), (ponto[0],ponto[1]+20), (255,255,255))

		cv2.imshow("img", img)
		cv2.waitKey(0)

	def center(self,list):
		soma = [0,0]
		for point in list:
			for ind in [0,1]:
				soma[ind] += point[ind]
		return soma[0]/4, soma[1]/4

	def distance(self, p1,p0):
		distance = ((p1[0]-p0[0])**2+
					(p1[1]-p0[1])**2) ** (0.5)
		return distance


calibrador = calib(0.03,"output.avi")
calibrador.process()

