# Feb 1 2021
# A very brief script I used to do some calculations for the tables in
# the SciTech 2021 conference paper

inputs = {"desired_decodes": 2999700000,
          "unsuccessful": 2934654239,
          "potentially_successful": 2999700000}

print("Normal prob:", 1.0 - (inputs["unsuccessful"] / inputs["desired_decodes"]))
print("Better prob:", 1.0 - (inputs["unsuccessful"] / inputs["potentially_successful"]))
