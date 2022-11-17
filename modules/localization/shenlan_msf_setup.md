# How to Set up ShenLan MSF Localization modules

==TODO==

## 1 Mapping 

```bash
bash supplement/update_docker.sh

mainboard -d modules/localization/dag/dag_streaming_shenlan_msf_localization.dag 

mainboard -d modules/localization/dag/dag_streaming_shenlan_msf_visualizer.dag

cyber_recorder play -f data/bag/bj30_2wd_fq_loc.record.00000  -k /apollo/localization/pose
```

