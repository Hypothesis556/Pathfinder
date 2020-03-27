from multiprocessing import Process,Queue,Pipe
import TATM
import time

def pseudo_locomotion(queue):
    while True:
        print("locomotion is running")
        if not queue.empty():
            print("Locomotion module has recognized that queue is populated")
            if queue.get() == 1:
                print("kill signal received from TATM")
                break
        time.sleep(2)
    print("locomotion terminated sucessfully")
    
def shepherd(parent_conn):
    print("shepherd initialized and standing by.")
    if parent_conn.recv() == 1:
        queue.put(1)
        print("shepherd has received signal and populated queue")
    shepherd(parent_conn)

parent_conn,child_conn = Pipe()
queue = Queue()
p1 = Process(target=TATM.main, args=(child_conn,))
p2 = Process(target=pseudo_locomotion, args=(queue,))
p3 = Process(target=shepherd, args=(parent_conn,))
p1.start()
print("p1 started")
p2.start()
print("p2 started")
p3.start()
print("p3 started")