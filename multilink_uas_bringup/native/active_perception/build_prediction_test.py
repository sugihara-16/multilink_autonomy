"""Build a diagnostic executable without replacing any flight binary."""
import shlex,subprocess
from build import HERE,WS,BUILD,flags

def main():
    out=WS/'build/rovio-articulated'
    directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir'
    f=flags(directory/'flags.make');obj=out/'test_prediction.o'
    subprocess.run(['c++',*f['CXX_DEFINES'],f'-I{BUILD}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O1','-c',str(HERE/'test_articulated_prediction.cpp'),'-o',str(obj)],check=True)
    link=shlex.split((directory/'link.txt').read_text())
    link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link]
    link[link.index('-o')+1]=str(out/'test_prediction')
    subprocess.run(link,check=True);subprocess.run([out/'test_prediction'],check=True)

if __name__=='__main__':main()
