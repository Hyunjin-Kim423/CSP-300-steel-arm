using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class DetectionMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/Detection";
        public override string RosMessageName => k_RosMessageName;

        public int class_id;
        public string class_name = "";
        public double score;
        public string id = "";
        public BoundingBox2DMsg bbox = new BoundingBox2DMsg();
        public BoundingBox3DMsg bbox3d = new BoundingBox3DMsg();
        public MaskMsg mask = new MaskMsg();
        public KeyPoint2DArrayMsg keypoints = new KeyPoint2DArrayMsg();
        public KeyPoint3DArrayMsg keypoints3d = new KeyPoint3DArrayMsg();

        public DetectionMsg() { }

        public static DetectionMsg Deserialize(MessageDeserializer deserializer) => new DetectionMsg(deserializer);

        DetectionMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out class_id);
            deserializer.Read(out class_name);
            deserializer.Read(out score);
            deserializer.Read(out id);
            bbox = BoundingBox2DMsg.Deserialize(deserializer);
            bbox3d = BoundingBox3DMsg.Deserialize(deserializer);
            mask = MaskMsg.Deserialize(deserializer);
            keypoints = KeyPoint2DArrayMsg.Deserialize(deserializer);
            keypoints3d = KeyPoint3DArrayMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(class_id);
            serializer.Write(class_name);
            serializer.Write(score);
            serializer.Write(id);
            serializer.Write(bbox);
            serializer.Write(bbox3d);
            serializer.Write(mask);
            serializer.Write(keypoints);
            serializer.Write(keypoints3d);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
