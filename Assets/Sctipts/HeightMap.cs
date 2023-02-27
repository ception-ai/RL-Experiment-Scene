using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class HeightMap : MonoBehaviour
{
    // Mesh
    private Mesh mesh;
    private Vector3[] vertices;
    private int[] triangles;
    private Color[] colors;
    private bool updateColor = true;

    // Terrain
    public Terrain terrain;
    private int xSize, zSize;
    private float minTerrainHeight;
    private float maxTerrainHeight;

    // Player Indecator
    public GameObject player;

    // General Params
    public Gradient gradient;
    public Vector3 offset;

    void Start()
    {
        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;

        xSize = (int)terrain.terrainData.size.x;
        zSize = (int)terrain.terrainData.size.z;

        transform.position = terrain.transform.position;
        transform.position += offset;
        player.SetActive(true);
    }

    void Update()
    {
        CreateShape();
        if (updateColor)
        {
            ColorUpdate();
            updateColor = false;
        }
        UpdateMesh();
        UpdatePlayer();
    }

    

    void CreateShape()
    {
        vertices = new Vector3[(xSize + 1) * (zSize + 1)];

        for (int i = 0, z = 0; z <= zSize; z++)
        {
            for (int x = 0; x <= xSize; x++)
            {
                float y = GetHeightWorldCoords(x, z);
                vertices[i] += new Vector3(x, y, z);

                if (y > maxTerrainHeight)
                    maxTerrainHeight = y;
                if (y < minTerrainHeight)
                    minTerrainHeight = y;

                i++;
            }
        }

        triangles = new int[xSize * zSize * 6];

        int vert = 0;
        int tris = 0;

        for (int z = 0; z < zSize; z++)
        {
            for (int x = 0; x < xSize; x++)
            {
                triangles[tris + 0] = vert + 0;
                triangles[tris + 1] = vert + xSize + 1;
                triangles[tris + 2] = vert + 1;
                triangles[tris + 3] = vert + 1;
                triangles[tris + 4] = vert + xSize + 1;
                triangles[tris + 5] = vert + xSize + 2;

                vert++;
                tris += 6;
            }
            vert++;
        }
    }

    void UpdateMesh()
    {
        mesh.Clear();

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.colors = colors;

        mesh.RecalculateNormals();
    }

    private void ColorUpdate()
    {
        colors = new Color[vertices.Length];

        for (int i = 0, z = 0; z <= zSize; z++)
        {
            for (int x = 0; x <= xSize; x++)
            {
                float height = Mathf.InverseLerp(minTerrainHeight, maxTerrainHeight, vertices[i].y);
                colors[i] = gradient.Evaluate(height);
                i++;
            }
        }
    }
    
    float GetHeightWorldCoords(int x, int z)
    {
        Vector3 scale = terrain.terrainData.heightmapScale;
        return (float)terrain.terrainData.GetHeight((int)(x / scale.x), (int)(z / scale.z));
    }

    private void UpdatePlayer()
    {
        if (GameObject.Find("wheel_loader_DL300(Clone)") != null)
        {
            GameObject frontBody = GameObject.Find("wheel_loader_DL300(Clone)").transform.Find("FrontBody").gameObject;
            
            player.transform.position = frontBody.transform.position + offset;
            player.transform.rotation = frontBody.transform.rotation;

            player.transform.position += new Vector3(0,terrain.GetComponent<AGXUnity.Model.DeformableTerrain>().MaximumDepth,0);
        }
    }
}
