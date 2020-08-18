# -*- coding: UTF-8 -*-
import face_recognition
import cv2
import os
import serial

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

# main process
ser = serial.Serial()
#ser.port = "/dev/ttyUSB0"
ser.port = "COM22"
#ser.port = "/dev/ttyS2"
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
#ser.timeout = None          #block read
ser.timeout = 1            #non-block read
#ser.timeout = 2              #timeout block read
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2     #timeout for write

try: 
    ser.open()
except Exception as e:
    print ("error open serial port: %s" %str(e) )
    exit()


if ser.isOpen():
    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
                 #and discard all that is in buffer
        ser.write("HELLO WORLD\r\n".encode())
    except Exception as e1:
        print ("error open serial port: %s" %str(e1) )

else:
    print ("cannot open serial port ")

# 这是一个超级简单（但很慢）的例子，在你的网络摄像头上实时运行人脸识别
# PLEASE NOTE: This example requires OpenCV (the `cv2` library) to be installed only to read from your webcam.
# 请注意：这个例子需要安装OpenCV
# 具体的演示。如果你安装它有困难，试试其他不需要它的演示。
# 得到一个参考的摄像头# 0（默认）
video_capture = cv2.VideoCapture(0)
video_capture.open(0, cv2.CAP_DSHOW)
#video_capture.set(3, 320); 
#video_capture.set(4, 240); 

Video_WIDTH  = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
Video_HEIGHT = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("HELLO WIDTH%d LENGTH%d" %( Video_WIDTH, Video_HEIGHT ) )
# 加载示例图片并学习如何识别它。
path ="images"#在同级目录下的images文件中放需要被识别出的人物图
total_image=[]
total_image_name=[]
total_face_encoding=[]
s_DetectFace = 0
name = "Unkown"

face_cascade = cv2.CascadeClassifier(r'./haarcascade_frontalface_default.xml')

for fn in os.listdir(path): #fn 表示的是文件名
  total_face_encoding.append(face_recognition.face_encodings(face_recognition.load_image_file(path+"/"+fn))[0])
  fn=fn[:(len(fn)-4)]#截取图片名（这里应该把images文件中的图片名命名为为人物名）
  total_image_name.append(fn)#图片名字列表
  s_DetectFace = 0
while True:
  # 抓取一帧视频
  ret, frame = video_capture.read()
  frame = cv2.flip(frame, 1)
  #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  #faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

  # 发现在视频帧所有的脸和face_enqcodings
  face_locations = face_recognition.face_locations(frame, model='hog')
  face_encodings = face_recognition.face_encodings(frame, face_locations)
  # 在这个视频帧中循环遍历每个人脸
  for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
      # 看看面部是否与已知人脸相匹配。
      for i,v in enumerate(total_face_encoding):
          match = face_recognition.compare_faces([v], face_encoding,tolerance=0.5)
          name = "Unknown"
          if match[0]:
              name = total_image_name[i]
              break
      # 画出一个框，框住脸
      s_DetectFace = 1
      print("START%d,%d,%d,%d,%dEND" %(left,top,right,bottom, right-left) )
      ser.flushInput() #flush input buffer, discarding all its contents
      ser.flushOutput()#flush output buffer, aborting current output 
      ser.write( ("START%d,%d,%d,%d,%dEND\r\n" %(left,top,right,bottom, right-left) ).encode() )
      cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), -1)
      # 画出一个带名字的标签，放在框下
      #cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
      #font = cv2.FONT_HERSHEY_DUPLEX
      #cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
  if (s_DetectFace == 0):
      print("START%d,%d,%d,%d,%dEND" %(0,0,0,0, 0) )
      ser.write( ("START%d,%d,%d,%d,%dEND\r\n" %(0,0,0,0, 0) ).encode() )
  else:
     s_DetectFace = 0
  # 显示结果图像
  cv2.imshow('Video', frame)
  # 按q退出
  if cv2.waitKey(1) & 0xFF == ord('q'):
      break
# 释放摄像头中的流
video_capture.release()
cv2.destroyAllWindows()
