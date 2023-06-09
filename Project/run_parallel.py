import threading
import subprocess
from time import sleep
from alive_progress import alive_bar

max_threads = 3
iterations = 10

def run_process(*args):
    cmd = ["python2", "run.py", "assignment.py"] + list(args)
    subprocess.call(cmd)
    

def base_process(file=None, dt=0.05, add=False):
    threads = []
    processed = 0
    if not file:
        file = "data/base_time.txt"
    if not add:
        f = open(file, "w+")
        f.write("Data with dt = {}: (Execution time, Inactivity time)\n".format(dt))
        f.close()
    with alive_bar(iterations) as bar:
        while processed < iterations:
            while len(threads) < max_threads and len(threads) < iterations - processed:
                t = threading.Thread(target=run_process, args=(["-f", file, "-t", str(dt)]))
                threads.append(t)
                t.start()
            
            i = 0
            while i < len(threads):
                if not threads[i].is_alive():
                    del threads[i]
                    processed += 1
                    print(processed)
                    bar()
                else:
                    i += 1
                    
            sleep(0.2)
            
            
# for t in range(15, 50, 5):
#     base_process("data/base_{}.txt".format(t), t/100.0)
base_process("data/aux_10.txt", 0.10, False)