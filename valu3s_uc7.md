# valu3s_uc7
ssh root@172.17.6.97 -p 12224
cd /isaac_sim/VALU3S/valu3s_uc7

rostest valu3s_uc7 test_uc7_ulises_fake_result.test 
- test_uc7_func_normal_ulises_fake_result.py
- test_uc7_door_not_openning_ulises_fake_result.py

rostest valu3s_uc7 test_dana_2.test 
  - test_uc7_bat.py
  - test_uc7_bi.py

rostest valu3s_uc7 test_uc7.test 
-  test_uc7_func_normal.py
- test_uc7_door_not_opening.py

rostest valu3s_uc7 test_uc7_door_not_openning.test 
- test_uc7_door_not_opening.py




# valu3s_uc7 new simualtion

ssh root@172.17.6.97 -p 12225
