import QtQuick 2.9
import QtQuick.Controls 2.5
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3

Rectangle {
  implicitWidth: 340
  implicitHeight: contentLayout.implicitHeight + 20
  color: "transparent"

  function formatValue(value) {
    return Number(value).toFixed(2)
  }

  function readNumber(text) {
    var parsed = Number(text)
    return isFinite(parsed) ? parsed : NaN
  }

  function syncFields() {
    if (!gridXField.activeFocus) {
      gridXField.text = formatValue(UavSpawner.gridSpacingX)
    }
    if (!gridYField.activeFocus) {
      gridYField.text = formatValue(UavSpawner.gridSpacingY)
    }
    if (!gridZField.activeFocus) {
      gridZField.text = formatValue(UavSpawner.gridSpacingZ)
    }
    if (!customXField.activeFocus) {
      customXField.text = formatValue(UavSpawner.customSpawnX)
    }
    if (!customYField.activeFocus) {
      customYField.text = formatValue(UavSpawner.customSpawnY)
    }
    if (!customZField.activeFocus) {
      customZField.text = formatValue(UavSpawner.customSpawnZ)
    }
  }

  Component.onCompleted: syncFields()

  Connections {
    target: UavSpawner

    function onPlacementChanged() {
      syncFields()
    }
  }

  ColumnLayout {
    id: contentLayout
    anchors.fill: parent
    anchors.margins: 10
    spacing: 4

    RowLayout {
      Layout.fillWidth: true

      Label {
        Layout.fillWidth: true
        text: "Next UAV: " + UavSpawner.nextName
        elide: Text.ElideRight
      }

      Label {
        text: UavSpawner.nextSpawnPoint
        horizontalAlignment: Text.AlignRight
        opacity: 0.8
        font.pixelSize: 14
      }
    }

    RowLayout {
      Layout.fillWidth: true

      CheckBox {
        enabled: !UavSpawner.busy
        text: "Grid spacing"
        checked: UavSpawner.useGridSpacing
        onToggled: UavSpawner.useGridSpacing = checked
      }

      Label {
        Layout.fillWidth: true
        text: UavSpawner.useGridSpacing ? "Grid pose inputs" : "Custom pose inputs"
        horizontalAlignment: Text.AlignRight
        opacity: 0.8
        font.pixelSize: 14
        elide: Text.ElideRight
      }
    }

    RowLayout {
      Layout.fillWidth: true
      visible: UavSpawner.useGridSpacing

      Label { text: "x: " }
      TextField {
        id: gridXField
        Layout.preferredWidth: 72
        enabled: !UavSpawner.busy
        validator: DoubleValidator {}
        onEditingFinished: {
          var parsed = readNumber(text)
          if (isNaN(parsed)) {
            text = formatValue(UavSpawner.gridSpacingX)
            return
          }
          UavSpawner.gridSpacingX = parsed
          text = formatValue(UavSpawner.gridSpacingX)
        }
      }

      Label { text: "y: " }
      TextField {
        id: gridYField
        Layout.preferredWidth: 72
        enabled: !UavSpawner.busy
        validator: DoubleValidator {}
        onEditingFinished: {
          var parsed = readNumber(text)
          if (isNaN(parsed)) {
            text = formatValue(UavSpawner.gridSpacingY)
            return
          }
          UavSpawner.gridSpacingY = parsed
          text = formatValue(UavSpawner.gridSpacingY)
        }
      }

      Label { text: "z: " }
      TextField {
        id: gridZField
        Layout.preferredWidth: 72
        enabled: !UavSpawner.busy
        validator: DoubleValidator {}
        onEditingFinished: {
          var parsed = readNumber(text)
          if (isNaN(parsed)) {
            text = formatValue(UavSpawner.gridSpacingZ)
            return
          }
          UavSpawner.gridSpacingZ = parsed
          text = formatValue(UavSpawner.gridSpacingZ)
        }
      }
    }

    RowLayout {
      Layout.fillWidth: true
      visible: !UavSpawner.useGridSpacing

      Label { text: "x: " }
      TextField {
        id: customXField
        Layout.preferredWidth: 72
        enabled: !UavSpawner.busy
        validator: DoubleValidator {}
        onEditingFinished: {
          var parsed = readNumber(text)
          if (isNaN(parsed)) {
            text = formatValue(UavSpawner.customSpawnX)
            return
          }
          UavSpawner.customSpawnX = parsed
          text = formatValue(UavSpawner.customSpawnX)
        }
      }

      Label { text: "y: " }
      TextField {
        id: customYField
        Layout.preferredWidth: 72
        enabled: !UavSpawner.busy
        validator: DoubleValidator {}
        onEditingFinished: {
          var parsed = readNumber(text)
          if (isNaN(parsed)) {
            text = formatValue(UavSpawner.customSpawnY)
            return
          }
          UavSpawner.customSpawnY = parsed
          text = formatValue(UavSpawner.customSpawnY)
        }
      }

      Label { text: "z: " }
      TextField {
        id: customZField
        Layout.preferredWidth: 72
        enabled: !UavSpawner.busy
        validator: DoubleValidator {}
        onEditingFinished: {
          var parsed = readNumber(text)
          if (isNaN(parsed)) {
            text = formatValue(UavSpawner.customSpawnZ)
            return
          }
          UavSpawner.customSpawnZ = parsed
          text = formatValue(UavSpawner.customSpawnZ)
        }
      }
    }

    RowLayout {
      Layout.fillWidth: true

      Button {
        Layout.fillWidth: true
        enabled: !UavSpawner.busy
        text: UavSpawner.busy ? "Spawning..." : "Spawn"
        onClicked: UavSpawner.Spawn()
      }

      Button {
        enabled: !UavSpawner.busy && UavSpawner.canRemoveLast
        text: "Remove last"
        font.pixelSize: 11
        onClicked: UavSpawner.RemoveLast()
      }
    }
  }
}
