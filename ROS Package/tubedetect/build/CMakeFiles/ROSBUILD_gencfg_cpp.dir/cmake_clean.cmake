FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/tubedetect/tubedetect_paramsConfig.h"
  "../docs/tubedetect_paramsConfig.dox"
  "../docs/tubedetect_paramsConfig-usage.dox"
  "../src/tubedetect/cfg/tubedetect_paramsConfig.py"
  "../docs/tubedetect_paramsConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
