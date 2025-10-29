# SIMDIS SDK 坐标系统深度分析

> **文档版本**: 1.0  
> **最后更新**: 2024  
> **作者**: 基于SIMDIS SDK源码分析  
> **License**: See LICENSE.txt

---

## 目录

1. [概述](#1-概述)
2. [坐标系统定义和数学基础](#2-坐标系统定义和数学基础)
3. [坐标转换矩阵和算法](#3-坐标转换矩阵和算法)
4. [SIMDIS SDK实体/传感器计算中的坐标系统使用](#4-simdis-sdk实体传感器计算中的坐标系统使用)
5. [实际代码实现细节](#5-实际代码实现细节)
6. [应用指南和性能优化](#6-应用指南和性能优化)
7. [参考资料](#7-参考资料)

---

## 1. 概述

### 1.1 SIMDIS SDK坐标系统架构

SIMDIS SDK (Simulation and Display Software Development Kit) 是由美国海军研究实验室 (NRL) 开发的仿真可视化SDK。它采用分层的坐标系统架构，支持从全球地理坐标到局部传感器坐标的完整转换链。

**核心设计理念**：
- **分层继承**：通过Locator继承链实现复杂的坐标系统层次
- **灵活转换**：支持多种坐标系统间的高精度转换
- **标准对齐**：默认使用NED (North-East-Down)局部坐标系，符合航空导航标准
- **传感器独立性**：支持绝对定位和机体相对两种模式

### 1.2 支持的实体类型

SIMDIS SDK支持以下主要实体类型，每种实体都有其特定的坐标系统特性：

| 实体类型 | 英文名称 | 主要用途 | 坐标系统特性 |
|---------|---------|---------|-------------|
| 平台 | Platform | 载体实体（飞机、舰船、车辆等） | 提供基准坐标系，宿主位置和姿态 |
| 波束 | Beam | 雷达/通信波束 | 支持BODY_RELATIVE和ABSOLUTE_POSITION两种模式 |
| 门限 | Gate | 雷达距离门 | 继承波束坐标系 |
| 激光 | Laser | 激光测距/指示器 | 支持相对平台姿态的角度测量 |
| 投影器 | Projector | 视频/图像投影 | 投影视场计算 |
| 方位线组 | LobGroup | 方位测量可视化 | 相对目标的方位显示 |
| 自定义渲染 | CustomRendering | 用户扩展实体 | 可自定义坐标系统 |

### 1.3 应用场景

- **雷达仿真**：波束指向、覆盖范围计算
- **激光系统**：测距、指示器指向
- **光学传感器**：视场计算、目标捕获
- **导航系统**：多平台位置关系计算
- **态势显示**：地理信息可视化

---

## 2. 坐标系统定义和数学基础

### 2.1 地理坐标系统

#### 2.1.1 大地坐标系 (LLA - Latitude, Longitude, Altitude)

**定义**: `COORD_SYS_LLA`

**坐标系参数**:
- **纬度 (Latitude)**: -π/2 ~ π/2 弧度 (-90° ~ 90°)
- **经度 (Longitude)**: -π ~ π 弧度 (-180° ~ 180°)
- **高度 (Altitude)**: 相对于椭球面的高度 (米)

**对齐方式**:
- 与ENU (East-North-Up) 系统对齐
- +X 轴方向：东 (East)
- +Y 轴方向：北 (North)
- +Z 轴方向：上 (Up)
- 航向 (Course) 绕Z轴顺时针

**参考椭球**: WGS-84

**代码示例**:
```cpp
// Coordinate.h:369
Vec3 pos_;  // position: radians and meter for geodetic
```

#### 2.1.2 WGS-84 椭球体参数

**定义位置**: `CoordinateSystem.h:64-77`

```cpp
// WGS-84 constants from NIMA TR8350.2, amendment 1, 3 Jan 2000
inline constexpr double WGS_A   = 6378137.0;                 // 长半轴 (m)
inline constexpr double WGS_E   = 0.081819190842622;         // 偏心率
inline constexpr double WGS_ESQ = 0.00669437999014;         // 偏心率平方
inline constexpr double WGS_F   = 1.0/298.257223563;        // 扁率
inline constexpr double WGS_B   = WGS_A * (1.0 - WGS_F);    // 短半轴 (m)
inline constexpr double WGS_B2  = WGS_B * WGS_B;            // 短半轴平方

// 地球相关常数
inline constexpr double EARTH_RADIUS = WGS_A;                    // 地球半径 (m)
inline constexpr double EARTH_ROTATION_RATE = 7292115.1467e-11;  // 地球自转角速度 (rad/sec)
```

**曲率半径公式** (用于缩放平面地球系统):

```cpp
// CoordinateConverter.cpp:344-353
const double sLat = sin(referenceOrigin_[0]);
const double x = 1.0 - WGS_ESQ * (sLat * sLat);
const double rN = WGS_A / sqrt(x);              // 卯酉圈曲率半径
const double latRadius = rN * (1. - WGS_ESQ) / x;   // 子午圈曲率半径
const double lonRadius = rN * cos(referenceOrigin_[0]);  // 纬圈曲率半径
```

### 2.2 地心坐标系

#### 2.2.1 地心地固坐标系 (ECEF - Earth Centered, Earth Fixed)

**定义**: `COORD_SYS_ECEF`

**坐标系参数**:
- X, Y, Z 坐标 (米)
- 右手笛卡尔坐标系

**坐标轴定义**:
- **原点**: 地球质心
- **+X轴**: 赤道平面内，指向格林威治子午线 (0°经度)
- **+Y轴**: 赤道平面内，指向90°东经
- **+Z轴**: 指向北极点 (旋转轴正方向)

**与LLA的转换关系**:

```cpp
// CoordinateConverter.cpp:2177-2186
void convertGeodeticPosToEcef(const Vec3 &llaPos, Vec3 &ecefPos, 
                               double semiMajor = WGS_A, 
                               double eccentricitySquared = WGS_ESQ)
{
  const double sLat = sin(llaPos.lat());
  const double Rn = semiMajor / sqrt(1.0 - eccentricitySquared * square(sLat));
  const double cLat = cos(llaPos.lat());

  ecefPos.set(
    (Rn + llaPos.alt()) * cLat * cos(llaPos.lon()),
    (Rn + llaPos.alt()) * cLat * sin(llaPos.lon()),
    (Rn * (1.0 - eccentricitySquared) + llaPos.alt()) * sLat
  );
}
```

#### 2.2.2 地心惯性坐标系 (ECI - Earth Centered Inertial)

**定义**: `COORD_SYS_ECI`

**坐标系参数**:
- X, Y, Z 坐标 (米)
- 惯性坐标系，不随地球自转

**特性**:
- 与ECEF使用相同的原点
- 初始时刻与ECEF重合
- 通过绕Z轴旋转实现转换
- 旋转角度 = ω × t，其中 ω 为地球自转角速度

**ECEF ↔ ECI 转换**:

```cpp
// CoordinateConverter.cpp:2058-2069
int convertEcefToEci(const Coordinate &ecefCoord, Coordinate &eciCoord)
{
  eciCoord.clear(COORD_SYS_ECI, ecefCoord.elapsedEciTime());
  convertEciEcef_(ecefCoord, eciCoord);  // 绕Z轴旋转
  return 0;
}

// 旋转角度计算
double rotationAngle = EARTH_ROTATION_RATE * elapsedTime;
// R = [cos(θ) -sin(θ) 0]
//     [sin(θ)  cos(θ) 0]
//     [0       0       1]
```

### 2.3 局部水平坐标系

#### 2.3.1 NED (North-East-Down)

**定义**: `COORD_SYS_NED`

**SIMDIS的主要局部坐标系**

**坐标轴定义**:
- **+X轴**: 指向北 (North)
- **+Y轴**: 指向东 (East)
- **+Z轴**: 指向下 (Down)

**应用场景**:
- 航空航天导航标准
- 机体姿态描述
- 局部地形坐标

**缩放平面地球投影**:
- 基于参考原点的曲率半径
- 近距离内保持准确的方向和距离
- 参考点靠近极点时退化

#### 2.3.2 ENU (East-North-Up)

**定义**: `COORD_SYS_ENU`

**坐标轴定义**:
- **+X轴**: 指向东 (East)
- **+Y轴**: 指向北 (North)
- **+Z轴**: 指向上 (Up)

**应用场景**:
- 地理信息系统 (GIS)
- 计算机视觉
- 某些传感器模型

#### 2.3.3 NWU (North-West-Up)

**定义**: `COORD_SYS_NWU`

**坐标轴定义**:
- **+X轴**: 指向北 (North)
- **+Y轴**: 指向西 (West)
- **+Z轴**: 指向上 (Up)

### 2.4 切平面坐标系

#### 2.4.1 X-East 切平面 (`COORD_SYS_XEAST`)

**定义**: 与ENU对齐的切平面

**特性**:
- 与地球表面相切于参考点
- 等距离线以原点为同心圆
- 距离原点越远，变形越大

#### 2.4.2 通用切平面 (`COORD_SYS_GTP`)

**特性**:
- 可自定义X-Y平面旋转角度
- 可设置X-Y偏移
- 适用于特殊投影需求

### 2.5 机体坐标系

**定义**: FRD (Forward-Right-Down) 系统

**坐标轴定义**:
- **+X轴**: 机体前向 (Forward)
- **+Y轴**: 机体右侧 (Right) 
- **+Z轴**: 机体下方 (Down)

**历史演变**:
```cpp
// Calculations.cpp:1360
// SIMDIS FLU body coordinates changed to a FRD system 
// in order to align to the NED frame
```

**与NED的对齐**:
- FRD坐标系与NED局部坐标系对齐
- 便于姿态角度的统一表示
- 机体的yaw/pitch/roll直接对应NED姿态

**姿态角定义** (Euler Angles):

```cpp
// CoordinateConverter.h:84-93
Yaw (ψ/psi):   绕惯性Z轴旋转，使惯性X轴在方位角上与机体X轴对齐
               正yaw = 右转 (右翼下沉)

Pitch (θ/theta): 绕新的惯性Y轴旋转，使惯性X轴与机体X轴对齐
                正pitch = 抬头 (nose up)

Roll (φ/phi):  绕新的惯性X轴旋转，使惯性Z轴与机体Z轴对齐
               正roll = 右翼下降 (顺时针)
```

**机体坐标转换**:

```cpp
// Calculations.cpp:1356-1363
// 将FLU (Forward-Left-Up) 转换为 FRD (Forward-Right-Down)
// 以对齐NED坐标系
Vec3 geoVec;
d3MTv3Mult(dcm, 
           Vec3(bodyPosOffset[0],      // X保持
                -bodyPosOffset[1],     // Y取反 (左→右)
                -bodyPosOffset[2]),    // Z取反 (上→下)
           geoVec);
```

---

## 3. 坐标转换矩阵和算法

### 3.1 欧拉角到方向余弦矩阵 (DCM)

#### 3.1.1 数学基础

DCM是一个 3×3 正交矩阵，用于将向量从一个坐标系旋转到另一个坐标系。

**转换序列** (从惯性系到机体系):
1. 绕Z轴旋转 ψ (yaw)
2. 绕新Y轴旋转 θ (pitch)
3. 绕新X轴旋转 φ (roll)

#### 3.1.2 公式实现

```cpp
// Dcm.cpp:105-151
void Dcm::fromEuler(const Vec3& ea)  // ea = [psi, theta, phi]
{
  const double spsi = sin(ea[0]);      // sin(psi)
  const double cpsi = cos(ea[0]);      // cos(psi)
  const double stheta = sin(ea[1]);    // sin(theta)
  const double ctheta = cos(ea[1]);    // cos(theta)
  const double sphi = sin(ea[2]);      // sin(phi)
  const double cphi = cos(ea[2]);     // cos(phi)

  // DCM矩阵元素 (3×3正交矩阵)
  set(0, 0, cpsi * ctheta);                         // R_xx
  set(0, 1, spsi * ctheta);                         // R_xy
  set(0, 2, -stheta);                                // R_xz
  
  set(1, 0, cpsi * stheta * sphi - spsi * cphi);    // R_yx
  set(1, 1, spsi * stheta * sphi + cpsi * cphi);    // R_yy
  set(1, 2, ctheta * sphi);                         // R_yz
  
  set(2, 0, cpsi * stheta * cphi + spsi * sphi);    // R_zx
  set(2, 1, spsi * stheta * cphi - cpsi * sphi);    // R_zy
  set(2, 2, ctheta * cphi);                         // R_zz
}
```

**完整DCM矩阵形式**:

```
     [cos(ψ)cos(θ)                  sin(ψ)cos(θ)                -sin(θ)        ]
DCM =[cos(ψ)sin(θ)sin(φ)-sin(ψ)cos(φ)  sin(ψ)sin(θ)sin(φ)+cos(ψ)cos(φ)  cos(θ)sin(φ)]
     [cos(ψ)sin(θ)cos(φ)+sin(ψ)sin(φ)  sin(ψ)sin(θ)cos(φ)-cos(ψ)sin(φ)  cos(θ)cos(φ)]
```

#### 3.1.3 DCM到欧拉角的逆变换

```cpp
// Dcm.cpp:61-103
Vec3 Dcm::toEuler() const
{
  Vec3 ea;
  
  // 处理万向锁 (gimbal lock)
  if (areEqual(get(0, 2), 1.0))  // pitch = -90°
  {
    ea.setV0(0.0);
    ea.setV1(-M_PI_2);
    ea.setV2(atan2(-get(1, 0), -get(2, 0)));
  }
  else if (areEqual(get(0, 2), -1.0))  // pitch = +90°
  {
    ea.setV1(M_PI_2);
    ea.setV2(atan2(get(1, 0), get(2, 0)));
    ea.setV0(0.0);
  }
  else  // 正常情况
  {
    ea.setV0(angFix2PI(atan2(get(0, 1), get(0, 0))));  // psi
    ea.setV1(inverseSine(-get(0, 2)));                   // theta
    ea.setV2(atan2(get(1, 2), get(2, 2)));               // phi
  }
  return ea;
}
```

### 3.2 局部到地球旋转矩阵

#### 3.2.1 Local-to-Earth 矩阵构建

用于将局部水平坐标系的向量转换为ECEF坐标系的向量。

**NED到ECEF的旋转矩阵**:

```cpp
// CoordinateConverter.cpp:1462-1520
void CoordinateConverter::setLocalToEarthMatrix(
    double lat, double lon, 
    LocalLevelFrame localLevelFrame, 
    double localToEarth[][3])
{
  const double slat = sin(lat);
  const double clat = cos(lat);
  const double slon = sin(lon);
  const double clon = cos(lon);

  switch (localLevelFrame) {
    case LOCAL_LEVEL_FRAME_NED:
      // NED局部系到ECEF地固系的单位向量
      localToEarth[0][0] = -slat * clon;  // NED X (North) 在ECEF中的单位向量
      localToEarth[0][1] = -slat * slon;
      localToEarth[0][2] =  clat;
      
      localToEarth[1][0] = -slon;        // NED Y (East) 在ECEF中的单位向量
      localToEarth[1][1] =  clon;
      localToEarth[1][2] =  0.0;
      
      localToEarth[2][0] = -clat * clon; // NED Z (Down) 在ECEF中的单位向量
      localToEarth[2][1] = -clat * slon;
      localToEarth[2][2] = -slat;
      break;
    // ... ENU, NWU cases
  }
}
```

**数学推导**:

对于NED系统，三个轴的单位向量在ECEF中的表示为:

```
North轴: [-sin(lat)cos(lon), -sin(lat)sin(lon), cos(lat)]
East轴:  [-sin(lon),          cos(lon),         0]
Down轴: [-cos(lat)cos(lon), -cos(lat)sin(lon), -sin(lat)]
```

#### 3.2.2 姿态转换流程

```cpp
// CoordinateConverter.cpp:1779-1794 (LLA到ECEF的姿态转换)
if (llaCoord.hasOrientation()) {
  double BL[3][3];  // Body to Local
  double BE[3][3];  // Body to Earth
  
  // 1. 将LLA姿态角转换为DCM
  d3EulertoDCM(llaCoord.orientation(), BL);
  
  // 2. 计算B to E (Body to Earth ECEF)
  d3MMmult(BL, LE, BE);  // BE = BL × LE
  
  // 3. 将DCM转换回欧拉角
  d3DCMtoEuler(BE, ecefOri);
  ecefCoord.setOrientation(ecefOri);
}
```

### 3.3 LLA ↔ ECEF 转换算法

#### 3.3.1 大地坐标到地心坐标 (LLA → ECEF)

**公式**:

```cpp
// CoordinateConverter.cpp:2177-2186
void convertGeodeticPosToEcef(const Vec3 &llaPos, Vec3 &ecefPos)
{
  const double sLat = sin(llaPos.lat());
  const double Rn = WGS_A / sqrt(1.0 - WGS_ESQ * sLat * sLat);
  const double cLat = cos(llaPos.lat());

  ecefPos.set(
    (Rn + llaPos.alt()) * cLat * cos(llaPos.lon()),
    (Rn + llaPos.alt()) * cLat * sin(llaPos.lon()),
    (Rn * (1.0 - WGS_ESQ) + llaPos.alt()) * sLat
  );
}
```

**数学表达式**:

```
N = a / √(1 - e²sin²(φ))                   卯酉圈曲率半径

x = (N + h) × cos(φ) × cos(λ)
y = (N + h) × cos(φ) × sin(λ)
z = (N × (1 - e²) + h) × sin(φ)
```

其中:
- φ = 纬度 (lat)
- λ = 经度 (lon)
- h = 高度 (alt)
- a = 长半轴 (WGS_A)
- e² = 偏心率平方 (WGS_ESQ)

#### 3.3.2 地心坐标到大地坐标 (ECEF → LLA)

使用 **Fukushima (1999)** 高效迭代算法:

```cpp
// CoordinateConverter.cpp:2074-2172
int convertEcefToGeodeticPos(const Vec3 &ecefPos, Vec3 &llaPos)
{
  // 经度计算 (直接计算)
  if (ecefPos.x() != 0.0) {
    llaPos.setLon(atan2(ecefPos.y(), ecefPos.x()));
  }
  
  // 迭代计算纬度和高度
  const double p = sqrt(ecefPos.x() * ecefPos.x() + ecefPos.y() * ecefPos.y());
  double S = ecefPos.z();
  double C = p;
  
  // Fukushima迭代 (最多2次迭代)
  for (int iter = 0; iter < 2; iter++) {
    Cc = C * FUKUSHIMA_eP;
    const double lat = sign(ecefPos.z()) * atan(S / Cc);
    
    const double S_n = fabs(ecefPos.z()) / sqrt(C * C + S * S);
    const double C_n = p / sqrt(C * C + S * S);
    
    C = C_n;
    S = S_n * ecefPos.z();
    
    // 收敛检查
    if (iter > 0 && (S == 0.0 || sign(S) == sign(Cc))) break;
  }
  
  // 计算纬度和高度
  const double lat = sign(ecefPos.z()) * atan(S / Cc);
  const double num = Cc * p + fabs(ecefPos.z()) * S - WGS_B * sqrt(C * C + S * S);
  const double den = sqrt(Cc * Cc + S * S);
  llaPos.setLat(lat);
  llaPos.setAlt(num / den);
}
```

### 3.4 ECEF ↔ ECI 转换

#### 3.4.1 地球自转角度

```cpp
// 地球自转角速度
const double EARTH_ROTATION_RATE = 7.292115147e-5;  // rad/sec

// 旋转角度
double rotationAngle = EARTH_ROTATION_RATE * elapsedTime;
```

#### 3.4.2 旋转矩阵

```
     [ cos(θ)  -sin(θ)  0 ]
R_z =[ sin(θ)   cos(θ)  0 ]
     [   0        0     1 ]
```

其中 θ = ω × t (ω = 地球自转角速度)

#### 3.4.3 转换实现

```cpp
// CoordinateConverter.cpp:544-580 (隐式实现)
void convertEciEcef_(const Coordinate &inCoord, Coordinate &outCoord)
{
  // ECEF → ECI: 逆旋转 (减去旋转角)
  // ECI → ECEF: 正旋转 (加上旋转角)
  
  const double rotAngle = EARTH_ROTATION_RATE * inCoord.elapsedEciTime();
  const double cosRot = cos(rotAngle);
  const double sinRot = sin(rotAngle);
  
  const Vec3& inPos = inCoord.position();
  Vec3& outPos = outCoord.position();
  
  // 2D旋转 (仅在XY平面)
  outPos.set(
    cosRot * inPos.x() - sinRot * inPos.y(),
    sinRot * inPos.x() + cosRot * inPos.y(),
    inPos.z()
  );
}
```

### 3.5 机体坐标系转换

#### 3.5.1 FRD到NED转换

```cpp
// Calculations.cpp:1353-1368
void calculateGeodeticOffsetPos(const Vec3& llaBgnPos, 
                                 const Vec3& bodyOriOffset, 
                                 const Vec3& bodyPosOffset,
                                 Vec3& offsetLla)
{
  // 1. 从欧拉角构建DCM
  double dcm[3][3];
  d3EulertoDCM(bodyOriOffset, dcm);
  
  // 2. FLU到FRD转换并旋转到局部坐标系
  Vec3 geoVec;
  d3MTv3Mult(dcm, 
             Vec3(bodyPosOffset[0],      // Forward
                  -bodyPosOffset[1],     // Right (Y取反)
                  -bodyPosOffset[2]),    // Down (Z取反)
             geoVec);
  
  // 3. 构建Local-to-Earth矩阵
  double localToEarth[3][3];
  CoordinateConverter::setLocalToEarthMatrix(
    llaBgnPos.lat(), llaBgnPos.lon(), 
    LOCAL_LEVEL_FRAME_NED, 
    localToEarth);
  
  // 4. 转换到地心系
  Vec3 geoOffVec;
  d3MTv3Mult(localToEarth, geoVec, geoOffVec);
  
  // 5. LLA → ECEF
  Vec3 originGeo;
  convertGeodeticPosToEcef(llaBgnPos, originGeo);
  
  // 6. 应用偏移并转换回LLA
  Vec3 offsetGeo = originGeo + geoOffVec;
  convertEcefToGeodeticPos(offsetGeo, offsetLla);
}
```

---

## 4. SIMDIS SDK实体/传感器计算中的坐标系统使用

### 4.1 Locator继承链架构

Locator是SIMDIS中实现坐标系统层次的核心机制。

#### 4.1.1 基础概念

**Locator组件**:

```cpp
// Locator.h:95-104
enum Components {
  COMP_NONE        = 0,
  COMP_POSITION    = 1 << 0,  // 位置
  COMP_HEADING     = 1 << 1,  // 航向
  COMP_PITCH       = 1 << 2,  // 俯仰
  COMP_ROLL        = 1 << 3,  // 滚转
  COMP_ORIENTATION = COMP_HEADING | COMP_PITCH | COMP_ROLL,
  COMP_ALL         = COMP_POSITION | COMP_ORIENTATION,
};
```

**继承机制**:
- Locator可以从父Locator继承特定的组件
- 支持选择性继承 (只继承位置或姿态)
- 可以实现复杂的传感器安装关系

#### 4.1.2 ResolvedPositionOrientationLocator

**用途**: 保持平台的完整位置和姿态，用于机体相对传感器

```cpp
// Locator.h:488-505
class ResolvedPositionOrientationLocator : public Locator
{
  // 生成一个"已解析"的坐标系
  // - 基于父Locator的位置和姿态
  // - 被视为基础坐标位置
  // - 后续继承的Locator会获得相同的"已解析"位置
  // - 继承的姿态矩阵代表最终位置的局部切平面
};
```

**应用场景**: BODY_RELATIVE波束

#### 4.1.3 ResolvedPositionLocator

**用途**: 仅保持位置，过滤掉平台姿态

```cpp
// Locator.h:514-527
class ResolvedPositionLocator : public ResolvedPositionOrientationLocator
{
  // 生成一个"仅位置"的坐标系
  // - 基于父Locator的位置 (不含姿态)
  // - 后续继承的Locator会获得相同位置
  // - 继承的姿态信息被设置为单位矩阵
  // - 用于绝对指向的传感器 (如绝对目标)
};
```

**应用场景**: ABSOLUTE_POSITION波束

#### 4.1.4 CachingLocator

**用途**: 性能优化，缓存常用的LLA位置和姿态

```cpp
// Locator.h:433-475
class CachingLocator : public Locator
{
private:
  mutable simCore::Vec3 llaPositionCache_;
  mutable osgEarth::Util::Revision llaPositionCacheRevision_;
  mutable simCore::Vec3 llaOrientationCache_;
  mutable osgEarth::Util::Revision llaOrientationCacheRevision_;
};
```

### 4.2 支持的实体类型及其坐标系统特性

#### 4.2.1 Platform (平台)

**定义**: 载体实体，提供基准坐标系

**坐标系统特点**:
- 宿主实体的位置和姿态由用户数据提供
- 存储为ECEF坐标系 (内部表示)
- 可以转换为LLA用于显示
- 作为所有传感器的父坐标系

**Locator链**:
```
PlatformLocator (ECEF) 
  ↓
  (提供位置和姿态)
  ↓
子实体Locator (继承位置和/或姿态)
```

**示例**:
```cpp
// Platform.cpp:1244
simCore::Coordinate platCoord = getLocator()->getCoordinate();
// platCoord 存储为 ECEF 坐标系

// 获取 LLA 用于显示
simCore::Coordinate llaCoord;
platform->getLocator()->getCoordinate(&llaCoord, simCore::COORD_SYS_LLA);
```

#### 4.2.2 Beam (波束)

**定义**: 雷达/通信波束

**坐标类型**:
```cpp
// DataTypeProperties.h:41-45
enum class Type {
  ABSOLUTE_POSITION = 1,  // 绝对指向
  BODY_RELATIVE,          // 相对机体姿态
  TARGET                  // 指向目标
};
```

**Locator链构建** (关键代码):

```cpp
// Beam.cpp:340-359
beamOriginLocator_ = new Locator(hostLocator, Locator::COMP_ALL);

if (props.type() == BODY_RELATIVE) {
  // 机体相对模式:
  // ResolvedPositionOrientationLocator 保持平台位置和姿态
  // 波束的方向和偏移相对于平台姿态
  beamOrientationLocator_ = new ResolvedPositionOrientationLocator(
    beamOriginLocator_.get(), Locator::COMP_ALL);
} else {
  // 绝对位置模式:
  // ResolvedPositionLocator 仅保持位置，过滤掉平台姿态
  // 波束方向不相对平台姿态
  beamOrientationLocator_ = new ResolvedPositionLocator(
    beamOriginLocator_.get(), Locator::COMP_ALL);
}
setLocator(beamOrientationLocator_.get());
```

**工作原理**:

**BODY_RELATIVE模式**:
1. 波束原始指向数据 (azimuth, elevation, slant range)
2. 叠加到平台姿态 (从ResolvedPositionOrientationLocator)
3. 最终指向 = 平台姿态 + 传感器安装偏移 + 原始指向

**ABSOLUTE_POSITION模式**:
1. 波束原始指向数据
2. 不叠加平台姿态 (平台姿态被过滤)
3. 最终指向 = 传感器位置 + 原始指向

#### 4.2.3 Gate (门限)

**定义**: 雷达距离门

**坐标系统特点**:
- 继承波束的Locator
- 在波束轴线上定义距离范围
- 通常用于目标检测和距离测量

**示例**:
```cpp
// Gate.cpp:370-378
// Locator that establishes a coordinate system at the gate centroid
// of the gate, stripped of orientation
gateLocator = new ResolvedPositionLocator(beamLocator, COMP_ALL);
```

#### 4.2.4 Laser (激光)

**定义**: 激光测距/指示器

**坐标系统特点**:
- 支持相对平台姿态的角度测量
- 可配置 `azElRelativeToHostOri_` 标志
- 类似于Beam的定位机制

**关键特性**:
```cpp
// LaserProperties
bool azElRelativeToHostOri_;  // 方位角/仰角是否相对平台姿态
```

#### 4.2.5 Projector (投影器)

**定义**: 视频/图像投影器

**坐标系统特点**:
- 投影视场角计算
- 在地球表面或椭球面上的投影
- 支持透视投影

**投影计算**:
```cpp
// Projector.cpp:950-957
getLocator()->setCoordinate(projPosition, time, eciRefTime);
// 投影器位置更新
```

#### 4.2.6 LobGroup (方位线组)

**定义**: 方位测量可视化

**坐标系统特点**:
- 从观测平台到目标平台的方位线显示
- 支持距离限制
- 通常显示为地面投影线

#### 4.2.7 CustomRendering (自定义渲染)

**定义**: 用户扩展实体

**坐标系统特点**:
- 可自定义Locator继承链
- 提供完全的灵活性
- 开发者可以创建特殊的坐标系统需求

### 4.3 传感器测量到物理世界的完整转换流程

#### 4.3.1 转换链概览

```
传感器本体测量 (Az/El/Range)
  ↓ [1. 传感器安装矩阵]
传感器相对机体偏移坐标 (Body Frame)
  ↓ [2. 如果是BODY_RELATIVE: 叠加机体姿态DCM]
机体坐标系 (FRD)
  ↓ [3. 机体姿态欧拉角 → DCM]
局部NED坐标系
  ↓ [4. LocalToEarth矩阵 (lat, lon)]
地心地固ECEF坐标系
  ↓ [5. 可选: 转换到LLA用于显示]
地理坐标LLA
  ↓ [6. 可选: 如需惯性系计算]
ECI (通过旋转角 ω×t)
```

#### 4.3.2 详细转换步骤

**步骤1: 获取平台状态**

```cpp
// 获取平台位置 (ECEF或LLA)
simCore::Coordinate platformCoord;
platform->getLocator()->getCoordinate(&platformCoord, simCore::COORD_SYS_LLA);

// 获取平台姿态
Vec3 platformOri = platformCoord.orientation();  // Yaw, Pitch, Roll
```

**步骤2: 设置坐标转换器参考点**

```cpp
simCore::CoordinateConverter converter;
converter.setReferenceOrigin(platformCoord.lat(), 
                             platformCoord.lon(), 
                             platformCoord.alt());
```

**步骤3: 构建Locator继承链**

```cpp
// 对于BODY_RELATIVE传感器 (如雷达波束)
if (sensorType == BODY_RELATIVE) {
  // 创建继承平台完整姿态的Locator
  Locator* sensorLocator = new ResolvedPositionOrientationLocator(
    platformLocator, Locator::COMP_ALL);
  
  // 应用传感器安装偏移
  sensorLocator->setLocalOffsets(
    Vec3(sensorX, sensorY, sensorZ),     // 位置偏移 (m)
    Vec3(0, 0, 0)                        // 姿态偏移 (rad)
  );
}
```

**步骤4: 传感器测量数据转换**

```cpp
// 假设传感器测量值
double azimuth = 0.0;       // 方位角 (rad)
double elevation = M_PI/6;  // 仰角 (rad, 30°)
double range = 1000.0;      // 距离 (m)

// 计算目标点的LLA坐标
Vec3 targetLLA;
calculateGeodeticEndPoint(platformLLA, 
                           azimuth, 
                           elevation, 
                           range,
                           targetLLA);
```

**calculateGeodeticEndPoint** 实现:

```cpp
// Calculations.cpp:1384-1393
void calculateGeodeticEndPoint(const Vec3 &llaBgnPos, 
                                const double az, const double el, 
                                const double rng, 
                                Vec3 &llaEndPos)
{
  if (simCore::areEqual(rng, 0.)) {
    llaEndPos = llaBgnPos;
    return;
  }
  
  // 计算带偏移的终点位置
  calculateGeodeticOffsetPos(llaBgnPos, 
                              Vec3(az, el, 0),  // 指向方向
                              Vec3(rng, 0, 0),  // 距离
                              llaEndPos);
}
```

**步骤5: 转换为ECEF用于物理计算**

```cpp
// 转换目标位置到ECEF
simCore::Coordinate targetECEF;
converter.convert(
  simCore::Coordinate(simCore::COORD_SYS_LLA, targetLLA),
  targetECEF,
  simCore::COORD_SYS_ECEF
);

// 现在可以进行3D空间计算
Vec3 relativeVec = targetECEF.position() - platformECEF.position();
double distance = relativeVec.length();
```

### 4.4 波束和门限的坐标系统处理示例

#### 4.4.1 波束处理流程

```cpp
// Beam.cpp 关键代码分析

// 1. 获取波束更新数据
const simData::BeamUpdate& update;  // 包含 azimuth, elevation, range

// 2. 根据波束类型选择Locator策略
if (props.type() == BODY_RELATIVE) {
  // 机体相对: 波束指向会随着平台姿态变化
  beamOrientationLocator_ = new ResolvedPositionOrientationLocator(...);
} else {
  // 绝对指向: 波束方向独立于平台姿态
  beamOrientationLocator_ = new ResolvedPositionLocator(...);
}

// 3. 计算波束体积
BeamVolume volume(prefs, update);
// 内部计算波束的锥体范围

// 4. 波束中心线计算
BeamCenterLine centerLine;
// 计算波束轴线的端点位置 (ECEF)

// 5. 位置更新
beamLocator_->setCoordinate(beamOriginECEF, currentTime);
```

#### 4.4.2 门限处理流程

```cpp
// Gate.cpp 关键代码分析

// 1. 门限体积计算
GateVolume* volume = createNode_(prefs, update);
// 门限体积继承自波束的Locator

// 2. 门限中心 (centroid) 计算
GateCentroid centroid(beamLocator);
// 在波束轴线上的位置

// 3. 中心Locator建立 (用于测距结果)
Locator* centroidLocator = new ResolvedPositionLocator(
  gateLocator, Locator::COMP_POSITION);
// 仅继承位置，用于显示测量到的目标位置
```

---

## 5. 实际代码实现细节

### 5.1 关键类和接口

#### 5.1.1 simCore::Coordinate

**文件**: `SDK/simCore/Calc/Coordinate.h`

**功能**: 存储坐标数据容器

```cpp
class Coordinate {
  CoordinateSystem system_;      // 坐标系类型
  Vec3 pos_;                     // 位置向量
  Vec3 vel_;                     // 速度向量 (可选)
  Vec3 ori_;                     // 姿态向量 (可选)
  Vec3 acc_;                     // 加速度向量 (可选)
  double elapsedEciTime_;        // ECI经过时间
  bool hasVel_, hasOri_, hasAcc_; // 数据有效性标志
};
```

**关键方法**:
```cpp
// 设置位置
void setPositionLLA(double lat, double lon, double alt);
void setOrientation(double yaw, double pitch, double roll);

// 获取组件
double lat() const { return pos_.lat(); }
double yaw() const { return ori_.yaw(); }
```

#### 5.1.2 simCore::CoordinateConverter

**文件**: `SDK/simCore/Calc/CoordinateConverter.h/cpp`

**功能**: 坐标系统转换器

**关键方法**:

```cpp
class CoordinateConverter {
  // 设置参考原点 (用于局部坐标系)
  void setReferenceOrigin(double lat, double lon, double alt);
  
  // 坐标转换
  int convert(const Coordinate &inCoord, Coordinate &outCoord, 
              CoordinateSystem outSystem) const;
  
  // 静态转换函数
  static void convertGeodeticPosToEcef(const Vec3 &llaPos, Vec3 &ecefPos, ...);
  static int convertEcefToGeodeticPos(const Vec3 &ecefPos, Vec3 &llaPos);
  static void setLocalToEarthMatrix(double lat, double lon, 
                                     LocalLevelFrame llf, 
                                     double localToEarth[][3]);
};
```

**内部状态**:
```cpp
double latRadius_, lonRadius_;           // 曲率半径
double rotationMatrixNED_[3][3];        // NED旋转矩阵
double rotationMatrixENU_[3][3];         // ENU旋转矩阵
Vec3 referenceOrigin_;                   // 参考原点 (LLA)
```

#### 5.1.3 simVis::Locator

**文件**: `SDK/simVis/Locator.h/cpp`

**功能**: 坐标位置定位器

**关键方法**:

```cpp
class Locator {
  // 设置世界坐标
  void setCoordinate(const simCore::Coordinate& coord, 
                      double timestamp, 
                      double eciRefTime);
  
  // 获取Locator矩阵 (用于渲染)
  osg::Matrixd getLocatorMatrix(unsigned int components = COMP_ALL) const;
  
  // 获取位置
  bool getLocatorPosition(simCore::Vec3* out_position,
                           const simCore::CoordinateSystem& coordsys) const;
  
  // 设置父Locator
  void setParentLocator(Locator* parent, 
                         unsigned int compsToInherit = COMP_ALL);
  
  // 设置局部偏移
  void setLocalOffsets(const simCore::Vec3& pos, 
                        const simCore::Vec3& ori);
};
```

**继承层次**:
```
Locator (基类)
  ├── CachingLocator (缓存优化)
  ├── ResolvedPositionOrientationLocator (解析位置+姿态)
  └── ResolvedPositionLocator (仅解析位置)
```

#### 5.1.4 simVis::EntityNode

**文件**: `SDK/simVis/Entity.h`

**功能**: 所有实体的基类

**派生类**:
- `PlatformNode`
- `BeamNode`
- `GateNode`
- `LaserNode`
- `ProjectorNode`
- `LobGroupNode`
- `CustomRenderingNode`

**关键方法**:
```cpp
class EntityNode {
  Locator* getLocator() const;
  void setLocator(Locator* locator);
  
  virtual bool isActive() const = 0;
  virtual bool isVisible() const = 0;
};
```

### 5.2 核心转换函数实现

#### 5.2.1 convertGeodeticPosToEcef

**实现位置**: `CoordinateConverter.cpp:2177-2187`

**算法**: 直接公式转换

```cpp
void CoordinateConverter::convertGeodeticPosToEcef(
    const Vec3 &llaPos, Vec3 &ecefPos, 
    const double semiMajor = WGS_A, 
    const double eccentricitySquared = WGS_ESQ)
{
  const double sLat = sin(llaPos.lat());
  const double Rn = semiMajor / sqrt(1.0 - eccentricitySquared * square(sLat));
  const double cLat = cos(llaPos.lat());

  ecefPos.set(
    (Rn + llaPos.alt()) * cLat * cos(llaPos.lon()),
    (Rn + llaPos.alt()) * cLat * sin(llaPos.lon()),
    (Rn * (1.0-eccentricitySquared) + llaPos.alt()) * sLat
  );
}
```

**时间复杂度**: O(1)

#### 5.2.2 convertEcefToGeodeticPos

**实现位置**: `CoordinateConverter.cpp:2074-2172`

**算法**: Fukushima (1999) 高效迭代算法

**特点**:
- 通常在1次迭代内收敛
- 对于接近地心的点需要2次迭代
- 避免了数值不稳定性

```cpp
int CoordinateConverter::convertEcefToGeodeticPos(
    const Vec3 &ecefPos, Vec3 &llaPos)
{
  // 1. 计算经度 (简单)
  if (ecefPos.x() != 0.0) {
    llaPos.setLon(atan2(ecefPos.y(), ecefPos.x()));
  }
  
  // 2. 初始估计
  const double p = sqrt(ecefPos.x() * ecefPos.x() + ecefPos.y() * ecefPos.y());
  double S = ecefPos.z();
  double C = p;
  
  // 3. Fukushima迭代
  for (int iter = 0; iter < 2; iter++) {
    const double Cc = C * FUKUSHIMA_eP;
    const double S_n = fabs(ecefPos.z()) / sqrt(C * C + S * S);
    const double C_n = p / sqrt(C * C + S * S);
    
    C = C_n;
    S = S_n * ecefPos.z();
    
    // 收敛检查
    if (iter > 0 && (S == 0.0 || sign(S) == sign(Cc))) break;
  }
  
  // 4. 计算最终结果
  const double Cc = C * FUKUSHIMA_eP;
  const double lat = sign(ecefPos.z()) * atan(S / Cc);
  const double num = Cc * p + fabs(ecefPos.z()) * S - WGS_B * sqrt(C * C + S * S);
  const double den = sqrt(Cc * Cc + S * S);
  
  llaPos.setLat(lat);
  llaPos.setAlt(num / den);
  
  return 0;
}
```

**时间复杂度**: O(1) (固定2次迭代)

#### 5.2.3 setLocalToEarthMatrix

**实现位置**: `CoordinateConverter.cpp:1462-1520`

**用途**: 构建局部坐标系到ECEF的旋转矩阵

```cpp
void CoordinateConverter::setLocalToEarthMatrix(
    double lat, double lon, 
    LocalLevelFrame localLevelFrame, 
    double localToEarth[][3])
{
  const double slat = sin(lat);
  const double clat = cos(lat);
  const double slon = sin(lon);
  const double clon = cos(lon);

  switch (localLevelFrame) {
    case LOCAL_LEVEL_FRAME_NED:
      localToEarth[0][0] = -slat * clon;  // North X
      localToEarth[0][1] = -slat * slon;  // North Y
      localToEarth[0][2] =  clat;         // North Z
      
      localToEarth[1][0] = -slon;         // East X
      localToEarth[1][1] =  clon;         // East Y
      localToEarth[1][2] =  0.0;         // East Z
      
      localToEarth[2][0] = -clat * clon; // Down X
      localToEarth[2][1] = -clat * slon; // Down Y
      localToEarth[2][2] = -slat;        // Down Z
      break;
      
    // ENU, NWU cases...
  }
}
```

#### 5.2.4 d3EulertoDCM / d3DCMtoEuler

**实现位置**: `Dcm.cpp:105-151`

**用途**: 欧拉角与方向余弦矩阵互转

```cpp
void Dcm::fromEuler(const Vec3& ea)  // [psi, theta, phi]
{
  const double spsi = sin(ea[0]);
  const double cpsi = cos(ea[0]);
  const double stheta = sin(ea[1]);
  const double ctheta = cos(ea[1]);
  const double sphi = sin(ea[2]);
  const double cphi = cos(ea[2]);

  set(0, 0, cpsi * ctheta);
  set(0, 1, spsi * ctheta);
  set(0, 2, -stheta);
  
  set(1, 0, cpsi * stheta * sphi - spsi * cphi);
  set(1, 1, spsi * stheta * sphi + cpsi * cphi);
  set(1, 2, ctheta * sphi);
  
  set(2, 0, cpsi * stheta * cphi + spsi * sphi);
  set(2, 1, spsi * stheta * cphi - cpsi * sphi);
  set(2, 2, ctheta * cphi);
}
```

### 5.3 使用示例和最佳实践

#### 5.3.1 设置参考原点

```cpp
simCore::CoordinateConverter converter;

// 设置参考原点 (用于局部坐标系转换)
double refLat = 40.0 * M_PI / 180.0;  // 纬度 (rad)
double refLon = -75.0 * M_PI / 180.0;  // 经度 (rad)
double refAlt = 100.0;                  // 高度 (m)

converter.setReferenceOrigin(refLat, refLon, refAlt);
// ⚠️ 注意: 这是一个开销较大的操作，会重新计算旋转矩阵
```

#### 5.3.2 进行坐标转换

```cpp
// 1. 定义输入坐标 (LLA)
simCore::Coordinate llaCoord(
  simCore::COORD_SYS_LLA,
  Vec3(lat, lon, alt),
  0.0  // ECI时间
);

// 2. 转换到ECEF
simCore::Coordinate ecefCoord;
int result = converter.convert(llaCoord, ecefCoord, simCore::COORD_SYS_ECEF);

if (result == 0) {
  // 成功转换
  Vec3 ecefPos = ecefCoord.position();
  std::cout << "ECEF: (" << ecefPos.x() 
            << ", " << ecefPos.y() 
            << ", " << ecefPos.z() << ")" << std::endl;
}
```

#### 5.3.3 构建Locator继承链

```cpp
// 1. 创建平台Locator
simVis::Locator* platformLocator = new simVis::Locator();
platformLocator->setCoordinate(
  simCore::Coordinate(simCore::COORD_SYS_LLA, Vec3(lat, lon, alt)),
  currentTime
);

// 2. 创建波束Locator (BODY_RELATIVE)
simVis::Locator* beamOriginLocator = new simVis::Locator(
  platformLocator, 
  simVis::Locator::COMP_ALL
);

// 3. 如果是机体相对波束
simVis::Locator* beamOrientationLocator = 
  new simVis::ResolvedPositionOrientationLocator(
    beamOriginLocator, 
    simVis::Locator::COMP_ALL
  );

// 4. 应用传感器安装偏移
Vec3 offsetPos(0.5, 0.0, 0.0);  // 0.5米前向偏移
Vec3 offsetOri(0.0, 0.0, 0.0); // 无姿态偏移
beamOrientationLocator->setLocalOffsets(offsetPos, offsetOri);
```

#### 5.3.4 处理BODY_RELATIVE传感器

```cpp
// Beam.cpp:340-359 (简化版)
if (beamProperties.type() == BeamProperties::Type::BODY_RELATIVE) {
  // 使用ResolvedPositionOrientationLocator
  // 保持平台姿态，传感器偏移相对于平台
  beamOrientationLocator = new ResolvedPositionOrientationLocator(
    beamOriginLocator, 
    Locator::COMP_ALL
  );
} else {
  // 使用ResolvedPositionLocator
  // 过滤平台姿态，传感器指向绝对方向
  beamOrientationLocator = new ResolvedPositionLocator(
    beamOriginLocator, 
    Locator::COMP_ALL
  );
}
```

#### 5.3.5 完整示例: 计算雷达覆盖范围

```cpp
// 场景: 计算雷达波束在地面上的覆盖范围

// 1. 获取平台状态
simCore::Coordinate platformLLA;
platform->getLocator()->getCoordinate(&platformLLA, simCore::COORD_SYS_LLA);

// 2. 设置坐标转换器
simCore::CoordinateConverter converter;
converter.setReferenceOrigin(
  platformLLA.lat(), 
  platformLLA.lon(), 
  platformLLA.alt()
);

// 3. 定义波束参数
double azimuth = M_PI / 4;      // 45度方位角
double elevation = M_PI / 6;     // 30度仰角
double range = 50000.0;           // 50公里范围

// 4. 计算波束边缘点 (简化版)
std::vector<Vec3> coveragePoints;
for (double azimuthOffset = -M_PI / 3; azimuthOffset <= M_PI / 3; azimuthOffset += M_PI / 30) {
  for (double elevationOffset = -M_PI / 12; elevationOffset <= M_PI / 12; elevationOffset += M_PI / 60) {
    double az = azimuth + azimuthOffset;
    double el = elevation + elevationOffset;
    
    // 计算该方向的地面交点
    Vec3 endPointLLA;
    calculateGeodeticEndPoint(
      platformLLA.position(),
      az,
      el,
      range,
      endPointLLA
    );
    
    coveragePoints.push_back(endPointLLA);
  }
}

// 5. 转换为ECEF用于3D计算
for (auto& point : coveragePoints) {
  simCore::Coordinate ecefCoord;
  converter.convert(
    simCore::Coordinate(simCore::COORD_SYS_LLA, point),
    ecefCoord,
    simCore::COORD_SYS_ECEF
  );
  // 现在可以进行3D几何计算
}
```

---

## 6. 应用指南和性能优化

### 6.1 传感器计算的坐标转换策略

#### 6.1.1 选择合适的坐标系统进行计算

**何时使用ECEF**:
- ✅ 3D空间计算 (距离、交点)
- ✅ 多平台相对位置
- ✅ 高精度要求
- ❌ 避免用于地理表示

**何时使用LLA**:
- ✅ 地理显示和标注
- ✅ 地形相关的计算
- ✅ 用户交互
- ❌ 避免用于3D空间计算

**何时使用局部坐标系 (NED/ENU)**:
- ✅ 局部运动学计算
- ✅ 传感器几何计算
- ✅ 小范围计算
- ❌ 避免用于跨大范围计算

#### 6.1.2 最小化坐标转换次数

```cpp
// ❌ 不好的做法: 重复转换
for (const auto& target : targets) {
  // 每次都转换
  Coordinate lla = convertToLLA(target.ecef);
  double dist = calculateDistance(platformLLA, lla);
}

// ✅ 好的做法: 批量转换
std::vector<Coordinate> targetLLAs;
// 批量转换一次
for (const auto& target : targets) {
  targetLLAs.push_back(convertToLLA(target.ecef));
}
// 然后统一计算
for (const auto& lla : targetLLAs) {
  double dist = calculateDistance(platformLLA, lla);
}
```

#### 6.1.3 使用CachingLocator提升性能

```cpp
// 对于频繁获取LLA位置的传感器
simVis::Locator* cachingLocator = new simVis::CachingLocator();
cachingLocator->setCoordinate(platformCoord, currentTime);

// 后续调用会使用缓存
simCore::Vec3 llaPos;
cachingLocator->getLocatorPosition(&llaPos, simCore::COORD_SYS_LLA);
// 第二次调用会更快
cachingLocator->getLocatorPosition(&llaPos, simCore::COORD_SYS_LLA);
```

### 6.2 常见应用场景

#### 6.2.1 雷达波束指向计算

```cpp
// 已知: 平台LLA, 波束方位角和仰角
Vec3 platformLLA(lat, lon, alt);
double azimuth = M_PI / 3;      // 60度
double elevation = M_PI / 4;    // 45度
double maxRange = 100000.0;      // 100公里

// 计算波束轴线上不同距离的点
for (double range = 0; range <= maxRange; range += 1000) {
  Vec3 targetLLA;
  calculateGeodeticEndPoint(platformLLA, azimuth, elevation, range, targetLLA);
  
  // 转换为ECEF用于进一步处理
  Coordinate targetECEF;
  converter.convert(Coordinate(COORD_SYS_LLA, targetLLA),
                    targetECEF,
                    COORD_SYS_ECEF);
  
  // 检查地形高度、遮挡等
  checkTerrain(targetECEF.position());
}
```

#### 6.2.2 激光测距

```cpp
// Laser实体本身已经处理了坐标系
LaserNode* laser = new LaserNode(laserProperties, platformLocator, host);

// 获取激光的有效点
std::vector<osg::Vec3d> endPoints;
laser->getVisibleEndPoints(endPoints);

// endPoints 已经是 ECEF 坐标
for (const auto& point : endPoints) {
  // 直接用于3D计算
  Vec3 laserVector = point - platformECEF.position();
  double distance = laserVector.length();
  
  // 投射到目标
  detectTarget(point);
}
```

#### 6.2.3 目标相对位置计算

```cpp
// 计算两个平台之间的相对位置
Coordinate platform1ECEF, platform2ECEF;

// 获取平台位置
platform1->getLocator()->getPosition(&platform1ECEF.position(), COORD_SYS_ECEF);
platform2->getLocator()->getPosition(&platform2ECEF.position(), COORD_SYS_ECEF);

// 计算相对向量
Vec3 relativeVec = platform2ECEF.position() - platform1ECEF.position();
double distance = relativeVec.length();

// 转换为相对方位角 (在NED坐标系中)
CoordinateConverter converter;
converter.setReferenceOrigin(platform1LLA.lat(), platform1LLA.lon(), platform1LLA.alt());

Coordinate localCoord;
converter.convert(platform2ECEF, localCoord, COORD_SYS_NED);

Vec3 localPos = localCoord.position();
double azimuth = atan2(localPos.y(), localPos.x());  // NED中的方位
double elevation = asin(-localPos.z() / distance);
```

#### 6.2.4 视场范围计算 (Projector)

```cpp
// 投影器视场角计算
ProjectorNode* projector = new ProjectorNode(projectorProps, locator);

// 获取投影器坐标系
osg::Matrixd projMatrix = projector->getLocator()->getLocatorMatrix();

// 计算视场范围
double hFOV = projectorPrefs.horizontalFov();
double vFOV = projectorPrefs.verticalFov();

// 计算视场角的四个角点
std::vector<Vec3> fovCorners;
for (int i = 0; i < 4; i++) {
  double hAngle = (i % 2 == 0 ? -1 : 1) * hFOV / 2;
  double vAngle = (i < 2 ? -1 : 1) * vFOV / 2;
  
  // 在投影器局部坐标系中的方向
  Vec3 localDir(cos(vAngle) * cos(hAngle),
                cos(vAngle) * sin(hAngle),
                sin(vAngle));
  
  // 转换到ECEF
  Vec3 ecefDir;
  d3Mv3Mult(projMatrix, localDir, ecefDir);
  
  fovCorners.push_back(ecefDir);
}
```

### 6.3 性能优化建议

#### 6.3.1 避免频繁设置参考原点

```cpp
// ❌ 不好的做法
for (const auto& point : manyPoints) {
  converter.setReferenceOrigin(point.lat(), point.lon(), point.alt());
  // 每次调用都会重新计算旋转矩阵和曲率半径
}

// ✅ 好的做法
// 设置一次参考原点 (使用场景中心点或最常用点)
converter.setReferenceOrigin(sceneCenterLat, sceneCenterLon, 0.0);

// 对于需要局部精度的计算，使用不同的参考点
// 但仅在必要时切换
```

**性能影响**: `setReferenceOrigin` 调用:
- 计算曲率半径 (2次√计算)
- 构建旋转矩阵 (2个矩阵)
- 如果设置频繁，会成为瓶颈

#### 6.3.2 批量转换优化

```cpp
// 对于大量点需要转换的情况
std::vector<Coordinate> convertBatch(
  const std::vector<Coordinate>& input,
  CoordinateSystem targetSystem)
{
  std::vector<Coordinate> output;
  output.reserve(input.size());  // 预分配
  
  // 单次设置参考原点
  if (targetSystem == COORD_SYS_NED || targetSystem == COORD_SYS_ENU) {
    // 使用第一个点作为参考
    converter.setReferenceOrigin(
      input[0].lat(), input[0].lon(), input[0].alt());
  }
  
  // 批量转换
  for (const auto& coord : input) {
    Coordinate target;
    converter.convert(coord, target, targetSystem);
    output.push_back(target);
  }
  
  return output;
}
```

#### 6.3.3 缓存常用计算结果

```cpp
// 对于重复计算的几何量
class CachedGeometry {
  mutable std::optional<Vec3> cachedLLA_;
  mutable std::optional<Vec3> cachedECEF_;
  mutable osgEarth::Util::Revision revision_;
  
  Vec3 getLLA() const {
    if (cachedLLA_.has_value()) {
      return *cachedLLA_;
    }
    
    // 计算并缓存
    cachedLLA_ = convertToLLA(position_);
    return *cachedLLA_;
  }
};

// 使用Locator的缓存功能
simVis::Locator* locator = new simVis::CachingLocator();
// 自动缓存LLA位置和姿态
```

#### 6.3.4 使用静态转换函数

```cpp
// 对于简单的坐标转换，使用静态函数
// 避免创建CoordinateConverter对象

// ✅ 静态函数 (无状态)
Vec3 ecefPos;
CoordinateConverter::convertGeodeticPosToEcef(llaPos, ecefPos);

// ❌ 对象函数 (需要创建对象)
CoordinateConverter converter;
converter.convert(llaCoord, ecefCoord, COORD_SYS_ECEF);
```

#### 6.3.5 选择合适的数据结构

```cpp
// 使用Vec3而不是单独的double
// ✅ 好的做法
Vec3 position(lat, lon, alt);
position.x();  // 纬度
position.y();  // 经度
position.z();  // 高度

// ❌ 避免使用
double lat, lon, alt;  // 容易混淆
```

---

## 7. 参考资料

### 7.1 相关文件路径索引

#### 核心头文件
- `SDK/simCore/Calc/Coordinate.h` - 坐标数据容器
- `SDK/simCore/Calc/CoordinateSystem.h` - 坐标系统枚举和常数
- `SDK/simCore/Calc/CoordinateConverter.h` - 坐标转换器接口
- `SDK/simCore/Calc/Dcm.h` - 方向余弦矩阵
- `SDK/simCore/Calc/Math.h` - 数学工具函数
- `SDK/simVis/Locator.h` - 定位器基类

#### 实现文件
- `SDK/simCore/Calc/CoordinateConverter.cpp` - 坐标转换实现 (2276行)
- `SDK/simCore/Calc/Dcm.cpp` - DCM操作实现
- `SDK/simCore/Calc/Math.cpp` - 数学工具实现
- `SDK/simCore/Calc/Calculations.cpp` - 地理计算函数
- `SDK/simVis/Locator.cpp` - Locator实现
- `SDK/simVis/Beam.cpp` - 波束实现
- `SDK/simVis/Gate.cpp` - 门限实现

#### 实体文件
- `SDK/simVis/Platform.h/cpp` - 平台实体
- `SDK/simVis/Beam.h/cpp` - 波束实体
- `SDK/simVis/Gate.h/cpp` - 门限实体
- `SDK/simVis/Laser.h/cpp` - 激光实体
- `SDK/simVis/Projector.h/cpp` - 投影器实体
- `SDK/simVis/LobGroup.h/cpp` - 方位线组实体

### 7.2 代码示例程序列表

#### 基础示例
- `Examples/BasicViewer/` - 基本查看器
- `Examples/LocatorTest/` - Locator使用示例

#### 传感器示例
- `Examples/BeamTest/` - 波束测试
- `Examples/GateTest/` - 门限测试
- `Examples/Projectors/` - 投影器示例
- `Examples/LOBTest/` - 方位线组示例

#### 高级示例
- `Examples/AntennaPattern/` - 天线方向图
- `Examples/RFProp/` - 射频传播
- `Examples/RangeTool/` - 距离工具

### 7.3 数学参考文献

1. **WGS-84标准**
   - NIMA TR8350.2, amendment 1, 3 Jan 2000
   - NIMA 标准地球模型定义

2. **坐标转换算法**
   - Fukushima, T. (1999). "Fast transform from geocentric to geodetic coordinates"
   - 高效率的ECEF到LLA转换算法

3. **方向余弦矩阵**
   - Stevens, B.L. & Lewis, F.L. (2003). "Aircraft Control and Simulation"
   - ISBN 0-471-37145-9
   - 第26-29页: 欧拉角和DCM的定义

4. **地球模型**
   - WGS84椭球体参数
   - 曲率半径计算公式: http://www.oc.nps.edu/oc2902w/geodesy/radiigeo.pdf

### 7.4 相关标准和规范

- **NED坐标系**: 航空航天导航标准 (MIL-STD-XXX)
- **WGS-84**: 世界大地测量系统
- **ECEF/ECI**: GPS/GNSS标准定义

---

## 附录

### A. 坐标转换速查表

| 从 | 到 | 函数 | 复杂度 |
|---|----|------|-------|
| LLA | ECEF | `convertGeodeticPosToEcef()` | O(1) |
| ECEF | LLA | `convertEcefToGeodeticPos()` | O(1) |
| ECEF | ECI | `convertEcefToEci()` | O(1) |
| ECI | ECEF | `convertEciToEcef()` | O(1) |
| 欧拉角 | DCM | `d3EulertoDCM()` | O(1) |
| DCM | 欧拉角 | `d3DCMtoEuler()` | O(1) |

### B. 常量速查

```cpp
// WGS-84
WGS_A   = 6378137.0 m              // 长半轴
WGS_B   = 6356752.3142 m           // 短半轴
WGS_E   = 0.081819190842622        // 偏心率
WGS_ESQ = 0.00669437999014         // 偏心率平方

// 地球旋转
EARTH_ROTATION_RATE = 7.292115147e-5 rad/sec  // 地球自转角速度
```

### C. 常见问题

**Q: 为什么SIMDIS使用NED而不是ENU?**  
A: NED是航空航天导航的标准坐标系，与机体坐标系(FRD)对齐，便于姿态表示。

**Q: ECI和ECEF有什么区别?**  
A: ECEF随着地球自转，ECI是惯性系不旋转。ECI用于轨道和天文计算。

**Q: 什么时候需要设置参考原点?**  
A: 仅在需要转换到/从局部坐标系(NED/ENU)时。参考原点应该是场景的中心点或最常用点。

**Q: BODY_RELATIVE和ABSOLUTE_POSITION的区别?**  
A: BODY_RELATIVE是传感器随平台姿态转动，ABSOLUTE_POSITION是传感器指向地球固定方向。

---

**文档结束**

