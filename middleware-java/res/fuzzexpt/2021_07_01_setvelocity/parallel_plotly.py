#!/usr/bin/env python3
import plotly.express as px
import plotly.io as pio
import pandas as pd
df = pd.read_csv("final-pop-without-empty.res")
pio.renderers.default="png"

fig = px.parallel_coordinates(df, color="ID",
                              labels={"OutsideOfOuterRegionViolation" : "OuterRegionV",
                                      "OutsideOfInnerRegionViolation" : "InnerRegionV",
                                      "AvoidanceViolation"            : "AvoidanceV",
                                      "FuzzingTimeLength"             : "TimeLen",
                                      "SpeedViolationsCount"          : "SpeedV" },
                              color_continuous_scale=px.colors.diverging.Tealrose,
                              color_continuous_midpoint=None)

fig.write_image("final-pop.png")

