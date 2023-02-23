using System.Linq;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

using AGXUnity.Utils;
using AGXUnityEditor;

using GUI = AGXUnity.Utils.GUI;

namespace AGXUnity_ExamplesWindow.Editor
{
  public class AGXUnityExamplesWindow : EditorWindow
  {
    public static readonly string DemoPackageName = "AGXDynamicsForUnityDemo";
    public static readonly string StandalonePackageName = "AGXUnity_ExamplesWindow";

    [MenuItem( "AGXUnity/Examples", priority = 5 )]
    public static AGXUnityExamplesWindow Open()
    {
      return GetWindow<AGXUnityExamplesWindow>( false,
                                                "AGX Dynamics for Unity Examples",
                                                true );
    }

    private void OnEnable()
    {
      var thumbnailDirectory = FindThumbnailDirectory();
      foreach ( var example in AGXUnityExamplesManager.Examples ) {
        if ( m_exampleIcons[ (int)example ] != null )
          continue;

        m_exampleIcons[ (int)example ] = EditorGUIUtility.Load( $"{thumbnailDirectory}/AGXUnity_{example}.png" ) as Texture2D;
        if ( m_exampleIcons[ (int)example ] == null )
          Debug.LogWarning( $"Unable to load preview image for example: {example}" );
      }
      AGXUnityExamplesManager.Initialize();
      EditorApplication.update += OnUpdate;
    }

    private void OnDisable()
    {
      EditorApplication.update -= OnUpdate;
      AGXUnityExamplesManager.Uninitialize();
    }

    private void OnGUI()
    {
      if ( m_exampleNameStyle == null ) {
        m_exampleNameStyle = new GUIStyle( InspectorEditor.Skin.Label );
        m_exampleNameStyle.alignment = TextAnchor.MiddleLeft;
      }

      using ( GUI.AlignBlock.Center )
        GUILayout.Box( IconManager.GetAGXUnityLogo(),
                       GUI.Skin.customStyles[ 3 ],
                       GUILayout.Width( 400 ),
                       GUILayout.Height( 100 ) );

      EditorGUILayout.LabelField( "© " + System.DateTime.Now.Year + " Algoryx Simulation AB",
                                  InspectorEditor.Skin.LabelMiddleCenter );

      InspectorGUI.BrandSeparator( 1, 6 );

      if ( EditorApplication.isPlayingOrWillChangePlaymode )
        ShowNotification( GUI.MakeLabel( "Playing..." ) );
      else if ( AGXUnityExamplesManager.IsInitializing )
        ShowNotification( GUI.MakeLabel( "Initializing..." ), 0.1 );
      else if ( EditorApplication.isCompiling )
        ShowNotification( GUI.MakeLabel( "Compiling..." ), 0.1 );
      else if ( AGXUnityExamplesManager.IsInstallingDependencies )
        ShowNotification( GUI.MakeLabel( "Installing..." ) );

      m_scroll = EditorGUILayout.BeginScrollView( m_scroll );

      bool hasDownloads = false;
      foreach ( var example in AGXUnityExamplesManager.Examples ) {
        var data = AGXUnityExamplesManager.GetData( example );

        using ( new EditorGUILayout.HorizontalScope() ) {
          GUILayout.Box( m_exampleIcons[ (int)example ],
                         GUILayout.Width( 64 ),
                         GUILayout.Height( 64 ) );
          var exampleNameLabel = GUI.MakeLabel( $"{example.ToString().SplitCamelCase()}", true );
          if ( data != null && data.Status == AGXUnityExamplesManager.ExampleData.State.Installed ) {
            if ( Link( exampleNameLabel, GUILayout.Height( 64 ) ) )
              Application.OpenURL( data.DocumentationUrl );
          }
          else
            GUILayout.Label( exampleNameLabel,
                             m_exampleNameStyle,
                             GUILayout.Height( 64 ) );

          GUILayout.FlexibleSpace();

          var hasUnresolvedIssues = AGXUnityExamplesManager.HasUnresolvedIssues( example );

          var buttonText = AGXUnityExamplesManager.IsInitializing ?
                             "Initializing..." :
                           data.Status == AGXUnityExamplesManager.ExampleData.State.Installed ?
                             "Load" :
                           data.Status == AGXUnityExamplesManager.ExampleData.State.Downloading ?
                             "Cancel" :
                           data.Status == AGXUnityExamplesManager.ExampleData.State.ReadyToInstall ||
                           data.Status == AGXUnityExamplesManager.ExampleData.State.Installing ?
                             "Importing..." :
                             "Install";
          var buttonEnabled = !AGXUnityExamplesManager.IsInitializing &&
                              !EditorApplication.isPlayingOrWillChangePlaymode &&
                              !EditorApplication.isCompiling &&
                              data != null &&
                              !hasUnresolvedIssues &&
                              !AGXUnityExamplesManager.IsInstallingDependencies &&
                              (
                                // "Install"
                                data.Status == AGXUnityExamplesManager.ExampleData.State.NotInstalled ||
                                // "Cancel"
                                data.Status == AGXUnityExamplesManager.ExampleData.State.Downloading ||
                                // "Load"
                                data.Status == AGXUnityExamplesManager.ExampleData.State.Installed
                              );
          using ( new GUI.EnabledBlock( buttonEnabled ) )
          using ( new EditorGUILayout.VerticalScope() ) {
            GUILayout.Space( 0.5f * ( 64 - 18 ) );
            if ( GUILayout.Button( GUI.MakeLabel( buttonText ),
                                   InspectorEditor.Skin.Button,
                                   GUILayout.Width( 120 ),
                                   GUILayout.Height( 18 ) ) ) {
              if ( data.Status == AGXUnityExamplesManager.ExampleData.State.NotInstalled )
                AGXUnityExamplesManager.Download( example );
              else if ( data.Status == AGXUnityExamplesManager.ExampleData.State.Downloading )
                AGXUnityExamplesManager.CancelDownload( example );
              else if ( data.Status == AGXUnityExamplesManager.ExampleData.State.Installed ) {
                if ( !string.IsNullOrEmpty( data.Scene ) )
                  EditorSceneManager.OpenScene( data.Scene, OpenSceneMode.Single );
                else
                  Debug.LogWarning( $"Unable to find .unity scene file for example: {example}" );
              }
            }
          }

          if ( data != null ) {
            if ( data.Status == AGXUnityExamplesManager.ExampleData.State.Downloading ) {
              hasDownloads = true;
              var progressBarWidth = 120.0f;
              var progressRect = GUILayoutUtility.GetLastRect();
              //progressRect.x -= ( 0.5f * ( 96 + progressBarWidth ) + 10.0f );
              //progressRect.y += 0.5f * ( 64 - 18 );
              progressRect.y += 4.0f;
              progressRect.height = 18.0f;
              progressRect.width = progressBarWidth;
              EditorGUI.ProgressBar( progressRect,
                                     data.DownloadProgress,
                                     $"Downloading: { (int)( 100.0f * data.DownloadProgress + 0.5f ) }%" );
            }
            else if ( hasUnresolvedIssues ) {
              var dependencyContextButtonWidth = 18.0f;
              var dependencyContextRect        = GUILayoutUtility.GetLastRect();
              dependencyContextRect.x         -= ( 0.5f * ( dependencyContextButtonWidth ) + 10.0f );
              dependencyContextRect.y         += 0.5f * ( 64 - 18 );
              dependencyContextRect.width      = dependencyContextButtonWidth;
              dependencyContextRect.height     = dependencyContextButtonWidth;

              var hasUnresolvedDependencies  = AGXUnityExamplesManager.HasUnresolvedDependencies( example );
              var hasUnresolvedInputSettings = ( data.RequiresLegacyInputManager &&
                                                 !AGXUnityExamplesManager.LegacyInputManagerEnabled ) ||
                                               ( data.Dependencies.Contains( AGXUnityExamplesManager.PackageDependency.InputSystem ) &&
                                                 !AGXUnityExamplesManager.InputSystemEnabled );
              var contextButton = InspectorGUI.Button( dependencyContextRect,
                                                       MiscIcon.ContextDropdown,
                                                       !AGXUnityExamplesManager.IsInstallingDependencies &&
                                                       !EditorApplication.isPlayingOrWillChangePlaymode,
                                                       ( hasUnresolvedDependencies ?
                                                           "Required dependencies." :
                                                           "Input settings has to be resolved." ),
                                                       1.1f );
              if ( contextButton ) {
                var dependenciesMenu = new GenericMenu();
                if ( hasUnresolvedDependencies ) {
                  dependenciesMenu.AddDisabledItem( GUI.MakeLabel( "Install dependency..." ) );
                  dependenciesMenu.AddSeparator( string.Empty );
                  foreach ( var dependency in data.Dependencies )
                    dependenciesMenu.AddItem( GUI.MakeLabel( dependency.ToString().SplitCamelCase() ),
                                              false,
                                              () => AGXUnityExamplesManager.InstallDependency( dependency ) );
                }
                else {
                  dependenciesMenu.AddDisabledItem( GUI.MakeLabel( "Resolve input settings..." ) );
                  dependenciesMenu.AddSeparator( string.Empty );
                  dependenciesMenu.AddItem( GUI.MakeLabel( "Enable both (legacy and new) Input Systems" ),
                                            false,
                                            () => AGXUnityExamplesManager.ResolveInputSystemSettings() );
                }
                dependenciesMenu.ShowAsContext();
              }
            }
          }
        }
        InspectorGUI.Separator();
      }

      EditorGUILayout.EndScrollView();

      if ( AGXUnityExamplesManager.IsInitializing || hasDownloads )
        Repaint();
    }

    private void OnUpdate()
    {
      AGXUnityExamplesManager.HandleImportQueue();
    }

    private static bool Link( GUIContent content, params GUILayoutOption[] options )
    {
      content.text = GUI.AddColorTag( content.text, EditorGUIUtility.isProSkin ?
                                                      InspectorGUISkin.BrandColorBlue :
                                                      Color.Lerp( InspectorGUISkin.BrandColorBlue,
                                                                  Color.black,
                                                                  0.20f ) );
      var clicked = GUILayout.Button( content, InspectorEditor.Skin.Label, options );
      EditorGUIUtility.AddCursorRect( GUILayoutUtility.GetLastRect(), MouseCursor.Link );
      return clicked;
    }

    private string FindThumbnailDirectory()
    {
      var script = MonoScript.FromScriptableObject( this );
      if ( script == null )
        return string.Empty;

      var scriptPath = AssetDatabase.GetAssetPath( script );
      if ( string.IsNullOrEmpty( scriptPath ) )
        return string.Empty;

      var path = scriptPath.Substring( 0, scriptPath.LastIndexOf( '/' ) + 1 ) + "Images";
      if ( System.IO.Directory.Exists( path ) )
        return path;
      return string.Empty;
    }

    [InitializeOnLoadMethod]
    private static void OpenOnInstall()
    {
      AssetDatabase.importPackageCompleted += OnThisPackageImported;
    }

    private static void OnThisPackageImported( string packageName )
    {
      var isDemoPackage = packageName.StartsWith( DemoPackageName );
      var isStandalonePackage = !isDemoPackage && packageName.StartsWith( StandalonePackageName );
      if ( isDemoPackage || isStandalonePackage ) {
        AssetDatabase.importPackageCompleted -= OnThisPackageImported;
        var data = EditorData.Instance.GetStaticData( "AGXUnityExamplesWindow_Open",
                                                      entry => entry.Bool = false );
        if ( !data.Bool ) {
          // Changing settings if this is the demo package. Warnings if standalone package.
          ValidateSettings( isDemoPackage );

          data.Bool = true;

          Open();
        }
      }
    }

    private static void ValidateSettings( bool change )
    {
      ValidateApiCompatibility( change );
      ValidateTimeManagerProperty( "Fixed Timestep", 0.02f, change );
      ValidateTimeManagerProperty( "Maximum Allowed Timestep", 0.02f, change );
    }

    private static bool ValidateApiCompatibility( bool change )
    {
      var isValid = PlayerSettings.GetApiCompatibilityLevel( BuildTargetGroup.Standalone ) == ApiCompatibilityLevel.NET_4_6;
      if ( isValid || !change )
        return isValid;

      PlayerSettings.SetApiCompatibilityLevel( BuildTargetGroup.Standalone, ApiCompatibilityLevel.NET_4_6 );
      Debug.Log( "Changed Project Setting:".Color( Color.green ) +
                 " .NET API compatibility level to " + ".NET 4.x".Color( Color.green ) + "." );

      return true;
    }

    private static bool ValidateTimeManagerProperty( string propertyName, float expectedValue, bool change )
    {
      var timeManagerObject = Unsupported.GetSerializedAssetInterfaceSingleton( "TimeManager" );
      if ( timeManagerObject == null )
        return true;

      var timeManager = new SerializedObject( timeManagerObject );
      var property = timeManager.FindProperty( propertyName );
      if ( property == null )
        return true;

      var currentValue = property.floatValue;
      var isValidValue = Math.Approximately( currentValue, expectedValue, 1.0E-4f );
      if ( isValidValue )
        return true;

      if ( !change ) {
        Debug.LogWarning( "Time Manager " +
                          propertyName.Color( Color.yellow ) +
                          " is currently " + currentValue.ToString( "0.00" ).Color( Color.yellow ) +
                          ", recommended value is " + expectedValue.ToString( "0.00" ).Color( Color.green ) + ".\n" +
                         $"<b>Edit -> Project Settings... -> Time -> {propertyName}</b>" );
        return false;
      }

      property.floatValue = expectedValue;
      timeManager.ApplyModifiedPropertiesWithoutUndo();

      Debug.Log( "Changed Project Setting:".Color( Color.green ) +
                $" {propertyName} from {currentValue.ToString( "0.00" )} to {expectedValue.ToString( "0.00" )}." );

      return true;
    }

    [ System.NonSerialized]
    private Texture2D[] m_exampleIcons = new Texture2D[ (int)AGXUnityExamplesManager.Example.NumExamples ];
    [System.NonSerialized]
    private GUIStyle m_exampleNameStyle = null;
    [SerializeField]
    private Vector2 m_scroll;
  }
}
