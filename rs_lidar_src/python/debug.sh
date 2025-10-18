pip uninstall -y rs_lidar
./clean_build.sh
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
./bdist_wheel.sh
pip install ./dist/*.whl
cd test
python test_pcap.py
cd ..