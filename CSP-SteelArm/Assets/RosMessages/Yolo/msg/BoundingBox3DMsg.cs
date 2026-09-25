using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class BoundingBox3DMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/BoundingBox3D";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PoseMsg center = new Geometry.PoseMsg();
        public Geometry.Vector3Msg size = new Geometry.Vector3Msg();
        public string frame_id = "";

        public BoundingBox3DMsg() { }

        public static BoundingBox3DMsg Deserialize(MessageDeserializer deserializer) => new BoundingBox3DMsg(deserializer);

        BoundingBox3DMsg(MessageDeserializer deserializer)
        {
            center = Geometry.PoseMsg.Deserialize(deserializer);
            size = Geometry.Vector3Msg.Deserialize(deserializer);
            deserializer.Read(out frame_id);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(center);
            serializer.Write(size);
            serializer.Write(frame_id);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
