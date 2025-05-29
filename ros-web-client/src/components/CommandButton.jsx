import React, { useContext } from "react";
import ROSLIB from "roslib";
import { RosContext } from "../ros/RosContext";

export default function CommandButton({ topic, label, messageData }) {
  const { ros } = useContext(RosContext);
  const handleClick = () => {
    if (!ros) return;
    const cmdTopic = new ROSLIB.Topic({ ros, name: topic, messageType: "std_msgs/String" });
    cmdTopic.publish(messageData);
  };
  return (
    <button className="btn btn-primary w-100 shadow-sm" onClick={handleClick} disabled={!ros}>
      {label}
    </button>
  );
}