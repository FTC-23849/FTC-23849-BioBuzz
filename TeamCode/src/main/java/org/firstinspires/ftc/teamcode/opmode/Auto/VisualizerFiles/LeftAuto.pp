{
  "startPoint": {
    "x": 64.5,
    "y": 132,
    "name": "startPos",
    "locked": true,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-mu4v1vxp-8iy8ld",
      "color": "#ffc516",
      "name": "Flower1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 48,
        "y": 124,
        "locked": false
      },
      "controlPoints": [
        {
          "x": 64.5,
          "y": 120
        },
        {
          "x": 48,
          "y": 100,
          "locked": true
        }
      ],
      "heading": {
        "type": "constant",
        "startDeg": 90,
        "endDeg": 180,
        "reverse": true,
        "degrees": 90
      }
    },
    {
      "id": "line-mvbeu4m2-m5ug80",
      "color": "#7B97B8",
      "name": "ReverseFlower1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 58,
        "y": 115,
        "locked": true
      },
      "controlPoints": [
        {
          "x": 48,
          "y": 115,
          "locked": true
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": true
      }
    },
    {
      "id": "line-mvbez3d8-zr85p9",
      "color": "#9C9C58",
      "name": "Flower2",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 15,
        "y": 48
      },
      "controlPoints": [
        {
          "x": 25.667874396135257,
          "y": 115
        },
        {
          "x": 35,
          "y": 48
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false
      }
    },
    {
      "id": "line-mvbfxpu0-g7dsuo",
      "color": "#77CA67",
      "name": "",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 58,
        "y": 115
      },
      "controlPoints": [
        {
          "x": 35,
          "y": 48
        },
        {
          "x": 26,
          "y": 115
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": true
      }
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 118.3,
          "y": 141.5
        },
        {
          "x": 135.5,
          "y": 118
        },
        {
          "x": 136.3,
          "y": 70.2
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.2,
          "y": 116.9
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-mu4v1vxp-8iy8ld"
    },
    {
      "kind": "path",
      "lineId": "line-mvbeu4m2-m5ug80"
    },
    {
      "kind": "path",
      "lineId": "line-mvbez3d8-zr85p9"
    },
    {
      "kind": "path",
      "lineId": "line-mvbfxpu0-g7dsuo"
    }
  ],
  "fieldPoints": [],
  "activePaths": [],
  "settings": {
    "xVelocity": 76.0579260801552,
    "yVelocity": 52.24299970011073,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.4,
    "rWidth": 18,
    "rHeight": 11.5,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "biobuzz.webp",
    "robotImage": "/robot.png",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false,
    "showHeadingArrow": true,
    "showCurrentTValue": false,
    "leftPanelWidth": 134,
    "rightPanelWidth": 682,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1,
    "leftPanelMinWidth": 0,
    "rightPanelMinWidth": 0,
    "penToolMaxPaths": 8,
    "experimentalFeatures": {
      "optimize": false,
      "curveThrough": false
    }
  },
  "version": "1.5.0",
  "timestamp": "2026-10-16T21:08:07.011Z"
}