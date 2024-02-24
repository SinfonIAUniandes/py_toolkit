import qi
import time

ss = qi.Session()
ss.connect("tcp://127.0.0.1:9559")
am = ss.service("ALMotion")
am.move(-1,0,0)
print("before")
time.sleep(10)
print("after")
am.stopMove()
print("stopped")
