import QtQuick 2.9
import QtQuick.Controls 2.2
Rectangle {
  width: 128; height: 48
  color: "#bb303030"
  Switch {
    anchors.centerIn: parent
    text: qsTr("光晕")
    contentItem: Text {
      text: parent.text
      color: "white"
      verticalAlignment: Text.AlignVCenter
      leftPadding: parent.indicator.width + parent.spacing
    }
    checked: SsbLightGlare.glareEnabled
    onToggled: SsbLightGlare.SetGlareEnabled(checked)
  }
}
