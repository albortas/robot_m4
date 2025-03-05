#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Arnaud Villeneuve

This file contains a class of functions to calculate the center of gravity position
and distance to the support polygon edge

"""

from math import sqrt
from src.utilities.cinematica import FK_Weight
from src.utilities.parametros import *


def CG_calculo (thetalf,thetarf,thetarr,thetalr):
    cgposlf=(FK_Weight(thetalf,1))
    cgposrf=(FK_Weight(thetarf,-1))
    cgposrr=(FK_Weight(thetarr,-1))
    cgposlr=(FK_Weight(thetalr,1))

    Weightsum = Weight_Body+4*(Weight_Shoulder+Weight_Leg+Weight_Foreleg)

    xcglf=(cgposlf[0]+xlf)*Weight_Shoulder+(cgposlf[1]+xlf)*Weight_Leg+(cgposlf[2]+xlf)*Weight_Foreleg
    xcgrf=(cgposrf[0]+xrf)*Weight_Shoulder+(cgposrf[1]+xrf)*Weight_Leg+(cgposrf[2]+xrf)*Weight_Foreleg
    xcgrr=(cgposrr[0]+xrr)*Weight_Shoulder+(cgposrr[1]+xrr)*Weight_Leg+(cgposrr[2]+xrr)*Weight_Foreleg
    xcglr=(cgposlr[0]+xlr)*Weight_Shoulder+(cgposlr[1]+xlr)*Weight_Leg+(cgposlr[2]+xlr)*Weight_Foreleg
    xcg= (xcglf+xcgrf+xcgrr+xcglr+xCG_Body*Weight_Body)/Weightsum

    ycglf=(cgposlf[3]+ylf)*Weight_Shoulder+(cgposlf[4]+ylf)*Weight_Leg+(cgposlf[5]+ylf)*Weight_Foreleg
    ycgrf=(cgposrf[3]+yrf)*Weight_Shoulder+(cgposrf[4]+yrf)*Weight_Leg+(cgposrf[5]+yrf)*Weight_Foreleg
    ycgrr=(cgposrr[3]+yrr)*Weight_Shoulder+(cgposrr[4]+yrr)*Weight_Leg+(cgposrr[5]+yrr)*Weight_Foreleg
    ycglr=(cgposlr[3]+ylr)*Weight_Shoulder+(cgposlr[4]+ylr)*Weight_Leg+(cgposlr[5]+ylr)*Weight_Foreleg
    ycg= (ycglf+ycgrf+ycgrr+ycglr+yCG_Body*Weight_Body)/Weightsum

    zcglf=(cgposlf[6]+zlf)*Weight_Shoulder+(cgposlf[7]+zlf)*Weight_Leg+(cgposlf[8]+zlf)*Weight_Foreleg
    zcgrf=(cgposrf[6]+zrf)*Weight_Shoulder+(cgposrf[7]+zrf)*Weight_Leg+(cgposrf[8]+zrf)*Weight_Foreleg
    zcgrr=(cgposrr[6]+zrr)*Weight_Shoulder+(cgposrr[7]+zrr)*Weight_Leg+(cgposrr[8]+zrr)*Weight_Foreleg
    zcglr=(cgposlr[6]+zlr)*Weight_Shoulder+(cgposlr[7]+zlr)*Weight_Leg+(cgposlr[8]+zlr)*Weight_Foreleg
    zcg= (zcglf+zcgrf+zcgrr+zcglr+zCG_Body*Weight_Body)/Weightsum

    return (xcg,ycg,zcg)


def CG_distance (x_legs,y_legs,z_legs,xcg,ycg,stance):

    #line equation c * x + s * y - p  = 0
    # with c = a/m et s = b/m

    a1 = (y_legs[0]-y_legs[2])
    b1 = -(x_legs[0]-x_legs[2])
    m1 =sqrt(a1**2 + b1**2)
    c1 = a1/m1
    s1 = b1/m1

    a2 = (y_legs[1]-y_legs[3])
    b2 = -(x_legs[1]-x_legs[3])
    m2 =sqrt(a2**2 + b2**2)
    c2 = a2/m2
    s2 = b2/m2

    p1 = c1*x_legs[0] + s1*y_legs[0]
    p2 = c2*x_legs[1] + s2*y_legs[1]

    """ Dstance calculation """
    d1 = c1*xcg + s1*ycg - p1
    d2 = c2*xcg + s2*ycg - p2

    """ intersection calculation """
    #perpendicalar line equation -s * x + c * y - q = 0

    q1 = -s1*xcg +c1*ycg
    q2 = -s2*xcg +c2*ycg

    xint1 = c1*p1 - s1*q1
    yint1 = c1*q1 + s1*p1

    xint2 = c2*p2 - s2*q2
    yint2 = c2*q2 + s2*p2

    """ Check if inside sustentation triangle """
    d = 0
    xint = xcg
    yint = ycg
    if (stance[0]== False)|(stance[2]== False):
        d = d2
        xint = xint2
        yint = yint2


    if (stance[1]== False)|(stance[3]== False):
        d = d1
        xint = xint1
        yint = yint1

    balance = True

    if (stance[0] == False)&(d< 0):
        balance = False

    if (stance[1] == False)&(d> 0):
        balance = False

    if (stance[2] == False)&(d> 0):
        balance = False

    if (stance[3] == False)&(d< 0):
        balance = False

    if (balance == False):
        d=-abs(d)
    else:
        d=abs(d)

    return (d,xint,yint,balance)
