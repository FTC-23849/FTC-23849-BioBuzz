{
  "startPoint": {
    "x": 53.5,
    "y": 9,
    "locked": true,
    "headingDeg": 90,
    "name": "startPos"
  },
  "lines": [
    {
      "id": "line-mvc2btrl-2fh915",
      "color": "#ffc516",
      "name": "Path 1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 12.813465783664457,
        "y": 47.06291390728478
      },
      "controlPoints": [
        {
          "x": 53.5,
          "y": 35
        },
        {
          "x": 35,
          "y": 48
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": -90,
        "endDeg": 180
      }
    },
    {
      "kind": "atomic",
      "id": "line-mvc2ilb4-w7y0al",
      "endPoint": {
        "x": 53.273730684326715,
        "y": 20.850993377483455
      },
      "controlPoints": [
        {
          "x": 34.073951434878595,
          "y": 45.8355408388521
        },
        {
          "x": 45.526799387442566,
          "y": 14.560490045941792
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": true
      },
      "color": "#9B6D88",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mvc2jrff-sravws",
      "endPoint": {
        "x": 10.212694677984784,
        "y": 4.909816807466973
      },
      "controlPoints": [
        {
          "x": 38.5811638591118,
          "y": 6.60566615620215
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false
      },
      "color": "#8DC7AC",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mvc2tffo-90wm9p",
      "endPoint": {
        "x": 55.714518489971574,
        "y": 19.523178807947033
      },
      "controlPoints": [
        {
          "x": 39.43032159264932,
          "y": 4.6921898928024435
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": true,
        "startDeg": 0,
        "endDeg": 0,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 0.7269272545282887,
              "interpolationType": "tangential",
              "reversed": true,
              "continueFromPrevious": false
            },
            {
              "startProgress": 0.7269272545282887,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "continueFromPrevious": true,
              "parameters": {
                "startDeg": 0,
                "endDeg": 90
              }
            }
          ]
        }
      },
      "color": "#DDC7D9",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mvc2yo3g-4cfx3v",
      "endPoint": {
        "x": 46.7665047378545,
        "y": 129.67947391729123
      },
      "controlPoints": [
        {
          "x": 58.32771822358345,
          "y": 133.1584992343032
        },
        {
          "x": 48.703379545585165,
          "y": 103.32997305693878
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false
      },
      "color": "#BDBB76",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mvc30dxb-xfsuc9",
      "endPoint": {
        "x": 7.385229996382805,
        "y": 111.22702656105798
      },
      "controlPoints": [
        {
          "x": 46.76743946262623,
          "y": 113.74672846329898
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": true
      },
      "color": "#AB7CDA",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
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
      "lineId": "line-mvc2btrl-2fh915"
    },
    {
      "kind": "path",
      "lineId": "line-mvc2ilb4-w7y0al"
    },
    {
      "kind": "path",
      "lineId": "line-mvc2jrff-sravws"
    },
    {
      "kind": "path",
      "lineId": "line-mvc2tffo-90wm9p"
    },
    {
      "kind": "path",
      "lineId": "line-mvc2yo3g-4cfx3v"
    },
    {
      "kind": "path",
      "lineId": "line-mvc30dxb-xfsuc9"
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
  "timestamp": "2026-10-17T10:52:21.634Z"
}