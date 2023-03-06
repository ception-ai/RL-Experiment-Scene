using System;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEditor.PackageManager.Requests;

using AGXUnityEditor.Web;

namespace AGXUnity_ExamplesWindow.Editor
{
  /// <summary>
  /// Manager handling download and import of examples and their
  /// optional dependencies, such as ML Agents, Input System and/or
  /// legacy Input Manager. On initialization the examples.html
  /// (of AGX Dynamics for Unity documentation) is parsed for
  /// download links and the current project is scanned for already
  /// installed examples. The status of dependent packages is also
  /// checked.
  /// </summary>
  public static class AGXUnityExamplesManager
  {
    /// <summary>
    /// Project package (Package Manager) dependencies for
    /// some examples.
    /// </summary>
    public enum PackageDependency
    {
      /// <summary>
      /// New Input System.
      /// </summary>
      InputSystem,
      /// <summary>
      /// ML Agents.
      /// </summary>
      MLAgents,
      /// <summary>
      /// Always last - number of dependencies.
      /// </summary>
      NumDependencies
    }

    /// <summary>
    /// Predefined examples - available or not.
    /// </summary>
    public enum Example
    {
      /// <summary>
      /// AGXUnity_Demo.unitypackage
      /// </summary>
      Demo,
      /// <summary>
      /// AGXUnity_WheelLoaderTerrain.unitypackage
      /// </summary>
      WheelLoaderTerrain,
      /// <summary>
      /// AGXUnity_WheelLoaderML.unitypackage
      /// </summary>
      WheelLoaderML,
      /// <summary>
      /// AGXUnity_RobotML.unitypackage
      /// </summary>
      RobotML,
      /// <summary>
      /// AGXUnity_DeckCrane.unitypackage
      /// </summary>
      DeckCrane,
      /// <summary>
      /// AGXUnity_GraspingRobot.unitypackage
      /// </summary>
      GraspingRobot,
      /// <summary>
      /// AGXUnity_ArticulatedRobot.unitypackage
      /// </summary>
      ArticulatedRobot,
      /// <summary>
      /// AGXUnity_Excavator.unitypackage
      /// </summary>
      Excavator,
      /// <summary>
      /// Always last - number of examples.
      /// </summary>
      NumExamples
    }

    /// <summary>
    /// Data of each example - may be null if the examples
    /// page hasn't been parsed (IsInitializing == true) or
    /// if the example has been removed.
    /// </summary>
    public class ExampleData
    {
      /// <summary>
      /// States of this example.
      /// </summary>
      public enum State
      {
        /// <summary>
        /// Unknown when the example exists but hasn't been
        /// checked whether it's installed.
        /// </summary>
        Unknown,
        /// <summary>
        /// The example is available for download but isn't
        /// installed in the project.
        /// </summary>
        NotInstalled,
        /// <summary>
        /// The example package is currently being downloaded.
        /// </summary>
        Downloading,
        /// <summary>
        /// The example package is available in the temporary
        /// directory but hasn't been imported.
        /// </summary>
        ReadyToInstall,
        /// <summary>
        /// The example is being imported into the project.
        /// </summary>
        Installing,
        /// <summary>
        /// The example is installed and valid in the project.
        /// </summary>
        Installed
      }

      /// <summary>
      /// Default directory name of the example.
      /// </summary>
      /// <param name="example">Example type.</param>
      /// <returns>Default directory name of the given example.</returns>
      public static string FindDirectoryName( Example example )
      {
        return $"AGXUnity_{example}";
      }

      /// <summary>
      /// Example this data is referring to.
      /// </summary>
      public Example Example = Example.NumExamples;

      /// <summary>
      /// Unique id parsed from the examples.html page.
      /// </summary>
      public string Id = string.Empty;

      /// <summary>
      /// Directory where the example is installed, null if not installed.
      /// </summary>
      public DirectoryInfo InstalledDirectory = null;

      /// <summary>
      /// Download progress [0 .. 1] if Status == State.Downloading, otherwise 0.
      /// </summary>
      public float DownloadProgress = 0.0f;

      /// <summary>
      /// Internal. Callback when the package has been downloaded used
      /// to abort current downloads.
      /// </summary>
      public Action<FileInfo, RequestHandler.Status> OnDownloadCompleteCallback = null;

      /// <summary>
      /// Internal. Reference to package to be imported.
      /// </summary>
      public FileInfo DownloadedPackage = null;

      /// <summary>
      /// Dependencies of the example.
      /// </summary>
      public PackageDependency[] Dependencies = new PackageDependency[] { };

      /// <summary>
      /// True if the example requires legacy input manager to be enabled.
      /// Some examples either use it for input or has EventSystem for other
      /// reasons. If True and disabled in Player Settings, Unity will throw
      /// exceptions.
      /// </summary>
      public bool RequiresLegacyInputManager = false;

      /// <summary>
      /// Current status of the example.
      /// </summary>
      public State Status { get; private set; } = State.Unknown;

      /// <summary>
      /// Download URL of the example package.
      /// </summary>
      public string DownloadUrl
      {
        get { return $@"https://us.download.algoryx.se/AGXUnity/documentation/current/_downloads/{Id}/{DirectoryName}.unitypackage"; }
      }

      /// <summary>
      /// Documentation URL of the example package.
      /// </summary>
      public string DocumentationUrl { get; set; } = string.Empty;

      /// <summary>
      /// Directory name (in project) of the example.
      /// </summary>
      public string DirectoryName => FindDirectoryName( Example );

      /// <summary>
      /// Scene file name including relative path to the project.
      /// </summary>
      public string Scene
      {
        get
        {
          if ( InstalledDirectory == null )
            return string.Empty;

          var sceneFileInfo = InstalledDirectory.EnumerateFiles( "*.unity",
                                                                 SearchOption.TopDirectoryOnly ).FirstOrDefault();
          if ( sceneFileInfo == null )
            return string.Empty;
          return AGXUnityEditor.IO.Utils.MakeRelative( sceneFileInfo.FullName,
                                                       Application.dataPath ).Replace( '\\', '/' );
        }
      }

      /// <summary>
      /// Internal. Update status of this example.
      /// </summary>
      /// <param name="status">New status.</param>
      public void UpdateStatus( State status )
      {
        Status = status;
      }

      /// <summary>
      /// Internal. Add dependency to the example.
      /// </summary>
      /// <param name="dependency">Dependency to add.</param>
      public void AddDependency( PackageDependency dependency )
      {
        if ( Dependencies.Contains( dependency ) )
          return;
        Dependencies = Dependencies.Concat( new PackageDependency[] { dependency } ).ToArray();
      }
    }

    /// <summary>
    /// Data of each dependency. This data is created during initialization.
    /// </summary>
    public class DependencyData
    {
      /// <summary>
      /// State of this dependency.
      /// </summary>
      public enum State
      {
        /// <summary>
        /// Unknown until the request has been received
        /// whether the dependency is installed or not.
        /// </summary>
        Unknown,
        /// <summary>
        /// The dependency is not installed.
        /// </summary>
        NotInstalled,
        /// <summary>
        /// The dependency is currently being installed.
        /// </summary>
        Installing,
        /// <summary>
        /// The dependency is installed.
        /// </summary>
        Installed
      }

      /// <summary>
      /// The dependency this data is referring to.
      /// </summary>
      public PackageDependency Dependency = PackageDependency.NumDependencies;

      /// <summary>
      /// Registry name of the dependency, e.g., com.unity.inputsystem.
      /// </summary>
      public string Name = string.Empty;

      /// <summary>
      /// Current status of the dependency.
      /// </summary>
      public State Status { get; private set; } = State.Unknown;

      /// <summary>
      /// Update status of the dependency.
      /// </summary>
      /// <param name="status">New status of the dependency.</param>
      public void UpdateStatus( State status )
      {
        Status = status;
      }
    }

    /// <summary>
    /// Enumerate examples.
    /// </summary>
    public static IEnumerable<Example> Examples
    {
      get
      {
        foreach ( Example example in Enum.GetValues( typeof( Example ) ) ) {
          if ( example == Example.NumExamples )
            yield break;
          else
            yield return example;
        }
      }
    }

    /// <summary>
    /// True when data is being fetched, otherwise false.
    /// </summary>
    public static bool IsInitializing { get; private set; } = false;

    /// <summary>
    /// True if dependencies are being installed, otherwise false.
    /// </summary>
    public static bool IsInstallingDependencies
    {
      get
      {
        return s_addPackageRequests.Count > 0;
      }
    }

    /// <summary>
    /// True if (new) Input System is enabled in the project settings,
    /// otherwise false.
    /// </summary>
    public static bool InputSystemEnabled
    {
      get
      {
        var property = InputSystemProperty;
        return property != null &&
               ( ( property.type == "bool" && property.boolValue ) ||
                 ( property.type == "int" && property.intValue >= 1 ) );
      }
    }

    /// <summary>
    /// True if (legacy) Input Manager is enabled in the project settings,
    /// otherwise false.
    /// </summary>
    public static bool LegacyInputManagerEnabled
    {
      get
      {
        var property = LegacyInputManagerDisabledProperty;
        return property == null ||
              ( property.type == "bool" && !property.boolValue ) ||
              ( property.type == "int" && property.intValue != 1 );
      }
    }

    /// <summary>
    /// Scraps old data and fetch new information about examples
    /// and dependencies.
    /// </summary>
    public static void Initialize()
    {
      IsInitializing = true;

      AssetDatabase.importPackageCompleted += OnImportPackageCompleted;
      AssetDatabase.importPackageCancelled += OnImportPackageCanceled;
      AssetDatabase.importPackageFailed    += OnImportPackageFailed;
      for ( int i = 0; i < s_exampleData.Length; ++i )
        s_exampleData[ i ] = null;
      for ( int i = 0; i < s_dependencyData.Length; ++i )
        s_dependencyData[ i ] = null;

      RequestHandler.Get( @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html",
                          TempDirectory,
                          OnExamplePage );
    }

    /// <summary>
    /// Abort any downloads and handling of currently importing dependencies.
    /// </summary>
    public static void Uninitialize()
    {
      AssetDatabase.importPackageCompleted -= OnImportPackageCompleted;
      AssetDatabase.importPackageCancelled -= OnImportPackageCanceled;
      AssetDatabase.importPackageFailed    -= OnImportPackageFailed;

      foreach ( var data in s_exampleData )
        RequestHandler.Abort( data?.OnDownloadCompleteCallback );
    }

    /// <summary>
    /// Resolve input settings to work with all examples. If any setting
    /// has been changed, the user will be asked to restart the editor.
    /// For all examples to work, both legacy input manager and new input
    /// system has to be enabled: Edit -> Project Settings -> Player ->
    /// Active Input Handling -> Both. This method is doing that for the caller.
    /// </summary>
    public static void ResolveInputSystemSettings()
    {
      // From ProjectSettings.asset:
      //     Old enabled:
      //       enableNativePlatformBackendsForNewInputSystem: 0
      //       disableOldInputManagerSupport: 0
      //     New enabled:
      //       enableNativePlatformBackendsForNewInputSystem: 1
      //       disableOldInputManagerSupport: 1
      //     Both:
      //       enableNativePlatformBackendsForNewInputSystem: 1
      //       disableOldInputManagerSupport: 0
      // In 2020.2 and later it's an int:
      //     activeInputHandler = 0: old, 1: new, 2 both
      // We want "Both".

      if ( InputSystemPropertyAsInt != null ) {
        if ( InputSystemPropertyAsInt.intValue == 2 )
          return;
        InputSystemPropertyAsInt.intValue = 2;
        InputSystemPropertyAsInt.serializedObject.ApplyModifiedPropertiesWithoutUndo();
      }
      else if ( InputSystemProperty != null && LegacyInputManagerDisabledProperty != null ) {
        if ( InputSystemProperty.boolValue && !LegacyInputManagerDisabledProperty.boolValue )
          return;
        InputSystemProperty.boolValue = true;
        LegacyInputManagerDisabledProperty.boolValue = false;

        InputSystemProperty.serializedObject.ApplyModifiedPropertiesWithoutUndo();
      }
      else
        return;

      var restartNow = EditorUtility.DisplayDialog( "Input System",
                                                    "Both Input System and (legacy) Input Manager must be enabled for " +
                                                    "all examples to work.\n\n" +
                                                    "This change has been applied to the Player Settings but requires restart of the editor. " +
                                                    "Restart the editor now?",
                                                    "Yes",
                                                    "Later" );
      if ( restartNow ) {
        var postRestartMethod = typeof( AGXUnityExamplesManager ).FullName + ".PostInputSystemSettingsRestart";
        Uninitialize();
        EditorApplication.OpenProject( Path.Combine( Application.dataPath, ".." ),
                                        "-executeMethod",
                                        postRestartMethod );
      }
    }

    /// <summary>
    /// Handle import queue where import of dependencies are
    /// prioritized. All examples has to be downloaded before
    /// the examples are imported.
    /// </summary>
    public static void HandleImportQueue()
    {
      if ( IsInitializing )
        return;

      if ( s_exampleData.Any( data => data != null &&
                                      ( data.Status == ExampleData.State.Installing ||
                                        data.Status == ExampleData.State.Downloading ) ) )
        return;

      // If we're installing dependencies we will not import any example.
      if ( s_addPackageRequests.Count > 0 ) {
        foreach ( var addRequest in s_addPackageRequests ) {
          if ( !addRequest.IsCompleted )
            continue;

          if ( addRequest.Status == StatusCode.Success ) {
            var dependencyData = s_dependencyData.FirstOrDefault( depData => depData != null &&
                                                                  depData.Name == addRequest.Result.name );
            if ( dependencyData != null ) {
              dependencyData.UpdateStatus( DependencyData.State.Installed );
              if ( dependencyData.Dependency == PackageDependency.InputSystem )
                ResolveInputSystemSettings();
            }
            else {
              Debug.Log( "No data for dependency :/" );
            }
          }
          else
            Debug.LogError( addRequest.Error.message );
        }

        s_addPackageRequests.RemoveAll( addRequest => addRequest.IsCompleted );

        return;
      }

      try {
        // Important for Unity to not start compilation of any
        // imported script. If Unity compiles, the data of the
        // downloaded packages will be deleted during Initialize
        // post-compile.
        AssetDatabase.DisallowAutoRefresh();

        foreach ( var data in s_exampleData ) {
          if ( data == null || data.DownloadedPackage == null )
            continue;

          if ( data.DownloadedPackage.Exists ) {
            data.UpdateStatus( ExampleData.State.Installing );
            AssetDatabase.ImportPackage( AGXUnityEditor.IO.Utils.MakeRelative( data.DownloadedPackage.FullName,
                                                                               Application.dataPath ).Replace( '\\', '/' ),
                                         false );
          }
          else
            data.DownloadedPackage = null;
        }
      }
      finally {
        AssetDatabase.AllowAutoRefresh();
      }
    }

    /// <summary>
    /// Get data of a given example. The data may be null if the data
    /// hasn't been parsed or if the example isn't available.
    /// </summary>
    /// <param name="example">Example to get data for.</param>
    /// <returns>Example data for the given example.</returns>
    public static ExampleData GetData( Example example )
    {
      return s_exampleData[ (int)example ];
    }

    /// <summary>
    /// Get data of a given dependency. The data may be null of the
    /// data hasn't been fetched or if the dependency isn't available.
    /// </summary>
    /// <param name="dependency">Dependency to get data for.</param>
    /// <returns>Dependency data for the given dependency.</returns>
    public static DependencyData GetData( PackageDependency dependency )
    {
      return s_dependencyData[ (int)dependency ];
    }

    /// <summary>
    /// True if there are issues with the current example. E.g.,
    /// unresolved dependencies or the project settings has to
    /// be changed.
    /// </summary>
    /// <param name="example">Example to check for issues with.</param>
    /// <returns>True if there are issues with the example, otherwise false.</returns>
    public static bool HasUnresolvedIssues( Example example )
    {
      var data = GetData( example );
      return HasUnresolvedDependencies( example ) ||
             data == null ||
             ( data.RequiresLegacyInputManager && !LegacyInputManagerEnabled ) ||
             ( data.Dependencies.Contains( PackageDependency.InputSystem ) && !InputSystemEnabled );
    }

    /// <summary>
    /// True if there are dependencies that hasn't been installed
    /// for the given example.
    /// </summary>
    /// <param name="example">Example to verify dependencies for.</param>
    /// <returns>True if there are unresolved dependencies for the example, otherwise false.</returns>
    public static bool HasUnresolvedDependencies( Example example )
    {
      var data = GetData( example );
      return data != null &&
             data.Dependencies.Length > 0 &&
             data.Dependencies.Any( dependency => s_dependencyData.Any( depData => depData != null &&
                                                                                   depData.Dependency == dependency &&
                                                                                   depData.Status != DependencyData.State.Installed ) );
    }

    /// <summary>
    /// Install the given dependency. Does nothing of the dependency
    /// is already available.
    /// </summary>
    /// <param name="dependency">Dependency to install.</param>
    public static void InstallDependency( PackageDependency dependency )
    {
      var depData = GetData( dependency );
      if ( depData == null )
        return;
      if ( depData.Status != DependencyData.State.NotInstalled )
        return;
      s_addPackageRequests.Add( Client.Add( depData.Name ) );
    }

    public static void Download( Example example )
    {
      var data = GetData( example );
      if ( data == null ) {
        Debug.LogWarning( $"Unable to download: {example} - data for the example isn't available." );
        return;
      }
      else if ( data.Status == ExampleData.State.Installed ) {
        Debug.LogWarning( $"Example {example} is already installed - ignoring download." );
        return;
      }
      else if ( data.Status == ExampleData.State.Downloading )
        return;

      if ( RequestHandler.Get( data.DownloadUrl,
                               TempDirectory,
                               OnDownloadComplete( data ),
                               OnDownloadProgress( data ) ) )
        data.UpdateStatus( ExampleData.State.Downloading );
    }

    /// <summary>
    /// Cancel download of the given example.
    /// </summary>
    /// <param name="example">Example to cancel download for.</param>
    public static void CancelDownload( Example example )
    {
      var data = GetData( example );
      if ( data == null || data.OnDownloadCompleteCallback == null )
        return;

      RequestHandler.Abort( data.OnDownloadCompleteCallback );
      data.UpdateStatus( ExampleData.State.NotInstalled );
    }

    /// <summary>
    /// Temporary directory "Temp" in the parent directory
    /// of "Assets".
    /// </summary>
    private static DirectoryInfo TempDirectory
    {
      get
      {
        if ( s_tempDirectory == null ) {
          s_tempDirectory = new DirectoryInfo( "./Temp" );
          if ( !s_tempDirectory.Exists )
            s_tempDirectory.Create();
        }
        return s_tempDirectory;
      }
    }

    /// <summary>
    /// Player Settings as a serialized object.
    /// </summary>
    private static SerializedObject PlayerSettings
    {
      get
      {
        if ( s_playerSettings == null )
          s_playerSettings = new SerializedObject( Unsupported.GetSerializedAssetInterfaceSingleton( "PlayerSettings" ) );
        return s_playerSettings;
      }
    }

    /// <summary>
    /// New from 2020.2: activeInputHandler = (0: old, 1: new, 2: both).
    /// </summary>
    private static SerializedProperty InputSystemPropertyAsInt
    {
      get { return PlayerSettings?.FindProperty( "activeInputHandler" ); }
    }

    /// <summary>
    /// Player Settings (new) Input System property where
    /// boolValue may be changed.
    /// </summary>
    private static SerializedProperty InputSystemProperty
    {
      get
      {
        return PlayerSettings?.FindProperty( "enableNativePlatformBackendsForNewInputSystem" ) ??
               InputSystemPropertyAsInt;
      }
    }

    /// <summary>
    /// Player Settings (legacy) Input Manager property where
    /// boolValue may be changed.
    /// </summary>
    /// <remarks>
    /// The value is inverted: "disableOldInputManagerSupport".
    /// </remarks>
    private static SerializedProperty LegacyInputManagerDisabledProperty
    {
      get
      {
        return PlayerSettings?.FindProperty( "disableOldInputManagerSupport" ) ??
               InputSystemPropertyAsInt;
      }
    }

    /// <summary>
    /// Closure of current download complete callback. When the
    /// download is complete, this method will call OnDownloadComplete
    /// with ExampleData, FileInfo and request status.
    /// </summary>
    /// <param name="data">Example data.</param>
    /// <returns>Callback used by the download request.</returns>
    private static Action<FileInfo, RequestHandler.Status> OnDownloadComplete( ExampleData data )
    {
      Action<FileInfo, RequestHandler.Status> onComplete = ( fi, status ) =>
      {
        data.OnDownloadCompleteCallback = null;
        OnDownloadComplete( data, fi, status );
      };
      data.OnDownloadCompleteCallback = onComplete;
      return onComplete;
    }

    /// <summary>
    /// Callback when an example package has been downloaded (or failed).
    /// </summary>
    /// <param name="data">Example data.</param>
    /// <param name="fi">File info to the downloaded package.</param>
    /// <param name="status">Request status.</param>
    private static void OnDownloadComplete( ExampleData data, FileInfo fi, RequestHandler.Status status )
    {
      lock ( s_downloadCompleteLock ) {
        if ( status == RequestHandler.Status.Success ) {
          data.UpdateStatus( ExampleData.State.ReadyToInstall );
          data.DownloadedPackage = fi;
        }
        else
          data.UpdateStatus( ExampleData.State.NotInstalled );
        data.DownloadProgress = 0.0f;
      }
    }

    /// <summary>
    /// Closure updating the ExampleData.DownloadProgress of the given
    /// example data during downloads.
    /// </summary>
    /// <param name="data">Example data of the example being downloaded.</param>
    /// <returns>The request OnProgress callback.</returns>
    private static Action<float> OnDownloadProgress( ExampleData data )
    {
      Action<float> onProgress = progress =>
      {
        data.DownloadProgress = progress;
      };
      return onProgress;
    }

    /// <summary>
    /// Callback hook for successfully imported packages.
    /// </summary>
    /// <param name="packageName">Any successfully imported package.</param>
    private static void OnImportPackageCompleted( string packageName )
    {
      OnImportPackageDone( packageName, "Success" );
    }

    /// <summary>
    /// Callback hook for imports being canceled.
    /// </summary>
    /// <param name="packageName">Any canceled package import.</param>
    private static void OnImportPackageCanceled( string packageName )
    {
      OnImportPackageDone( packageName, "Canceled" );
    }

    /// <summary>
    /// Callback hook for failed imports.
    /// </summary>
    /// <param name="packageName">Any failed package.</param>
    /// <param name="error">Error of the import (ignored, assumed to be printed in the Console).</param>
    private static void OnImportPackageFailed( string packageName, string error )
    {
      OnImportPackageDone( packageName, "Failed" );
    }

    /// <summary>
    /// Updating status of the example being imported to either
    /// Installed (if status == "Success") or NotInstalled for
    /// any other <paramref name="status"/>.
    /// </summary>
    /// <param name="packageName">Name of the package being handled.</param>
    /// <param name="status">Status "Success", "Failed" or "Canceled".</param>
    private static void OnImportPackageDone( string packageName, string status )
    {
      var exampleMatch = Regex.Match( packageName, @"AGXUnity_(\w+)" );
      if ( exampleMatch.Success &&
           Enum.TryParse( exampleMatch.Groups[ 1 ].ToString(),
                          out Example example ) ) {
        // Data can be null here if Unity has compiled scripts from the example.
        var data = GetData( example );
        if ( data != null ) {
          if ( status == "Success" ) {
            data.InstalledDirectory = new DirectoryInfo( $"Assets/{packageName}" );
            data.UpdateStatus( ExampleData.State.Installed );
          }
          else
            data.UpdateStatus( ExampleData.State.NotInstalled );
        }

        DeleteTemporaryPackage( example );
      }
    }

    /// <summary>
    /// Callback when Unity has been restarted due to changes in
    /// the Player Settings (that requires restart). This method
    /// is triggering a recompile of all scripts.
    /// </summary>
    private static void PostInputSystemSettingsRestart()
    {
      // Note: This define symbol isn't used, it's only used to
      //       trigger scripts to be recompiled when AGXUnity
      //       uses ENABLE_INPUT_SYSTEM define symbol.
      if ( AGXUnityEditor.Build.DefineSymbols.Contains( "AGXUNITY_INPUT_SYSTEM" ) )
        AGXUnityEditor.Build.DefineSymbols.Remove( "AGXUNITY_INPUT_SYSTEM" );
      else
        AGXUnityEditor.Build.DefineSymbols.Add( "AGXUNITY_INPUT_SYSTEM" );
    }

    /// <summary>
    /// Callback when the examples.html has been downloaded to the
    /// temporary directory. This method parses the file for download
    /// links and checks the project for installed examples. Last,
    /// this method fires a request for a list of installed packages
    /// in the project.
    /// </summary>
    /// <param name="file">Downloaded file.</param>
    /// <param name="status">Request status.</param>
    private static void OnExamplePage( FileInfo file, RequestHandler.Status status )
    {
      if ( status != RequestHandler.Status.Success )
        return;

      try {
        using ( var streamReader = file.OpenText() ) {
          var line = string.Empty;
          while ( ( line = streamReader.ReadLine() ) != null ) {
            var match = s_examplesPageRegex.Match( line );
            if ( !match.Success )
              continue;

            var id   = match.Groups[ 1 ].ToString();
            var name = match.Groups[ 2 ].ToString();
            if ( !Enum.TryParse( name, out Example example ) )
              continue;

            s_exampleData[ (int)example ] = new ExampleData()
            {
              Example = example,
              Id = id,
            };
          }
        }
      }
      finally {
        file.Delete();
      }

      for ( int i = 0; i < s_exampleData.Length; ++i ) {
        var data = s_exampleData[ i ];
        if ( data == null ) {
          Debug.LogWarning( $"No data for example: {(Example)i}" );
          continue;
        }

        var directories = Directory.GetDirectories( "Assets",
                                                    data.DirectoryName,
                                                    SearchOption.AllDirectories );
        data.InstalledDirectory = ( from dir in directories
                                    let di = new DirectoryInfo( dir )
                                    where di.EnumerateFiles( $"{data.DirectoryName}.unity",
                                                             SearchOption.TopDirectoryOnly ).FirstOrDefault() != null
                                    select di ).FirstOrDefault();
        if ( data.InstalledDirectory != null )
          data.UpdateStatus( ExampleData.State.Installed );
        else
          data.UpdateStatus( ExampleData.State.NotInstalled );

        // We don't want downloaded packages in the temp folder. E.g., if
        // the user deletes some example folder, it's not desired to automatically
        // install the package again when opening the examples window.
        DeleteTemporaryPackage( data.Example );

        if ( data.Example == Example.WheelLoaderTerrain ||
             data.Example == Example.GraspingRobot ||
             data.Example == Example.Excavator ||
             data.Example == Example.ArticulatedRobot )
          data.AddDependency( PackageDependency.InputSystem );

        if ( data.Example == Example.WheelLoaderML ||
             data.Example == Example.RobotML )
          data.AddDependency( PackageDependency.MLAgents );

        data.RequiresLegacyInputManager = data.Example == Example.RobotML ||
                                          data.Example == Example.DeckCrane;

        // This is rather explicit but the URL in the documentation
        // is generated given the header which is unknown here.
        if ( data.Example == Example.Demo )
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html#demo-scene";
        else if ( data.Example == Example.WheelLoaderTerrain )
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html#wheel-loader-on-terrain";
        else if ( data.Example == Example.WheelLoaderML )
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html#ml-agents-wheel-loader-way-point-controller";
        else if ( data.Example == Example.RobotML )
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html#ml-agents-robot-poking-box-controller";
        else if ( data.Example == Example.DeckCrane )
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html#deck-crane";
        else if ( data.Example == Example.GraspingRobot )
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html#grasping-robot";
        else if ( data.Example == Example.ArticulatedRobot )
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html#articulated-robot";
        else if ( data.Example == Example.Excavator )
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html#excavator-on-terrain";
        else {
          Debug.LogWarning( $"Missing explicit documentation URL for {data.Example}." );
          data.DocumentationUrl = @"https://us.download.algoryx.se/AGXUnity/documentation/current/examples.html";
        }
      }

      s_listPackagesRequest = Client.List( true );
      EditorApplication.update += InitializeDependencies;
    }

    /// <summary>
    /// An EditorApplication.update callback waiting for the list of
    /// currently installed packages in the project. The callback is
    /// removed when the request is completed.
    /// </summary>
    private static void InitializeDependencies()
    {
      if ( s_listPackagesRequest == null ) {
        EditorApplication.update -= InitializeDependencies;
        return;
      }

      if ( s_listPackagesRequest.IsCompleted ) {
        s_dependencyData[ (int)PackageDependency.InputSystem ] = new DependencyData()
        {
          Dependency = PackageDependency.InputSystem,
          Name = "com.unity.inputsystem"
        };
        s_dependencyData[ (int)PackageDependency.MLAgents ] = new DependencyData()
        {
          Dependency = PackageDependency.MLAgents,
          Name = "com.unity.ml-agents"
        };

        for ( int i = 0; i < s_dependencyData.Length; ++i ) {
          if ( s_dependencyData[ i ] == null )
            Debug.LogWarning( $"Unknown dependency: {(PackageDependency)i}" );
        }

        if ( s_listPackagesRequest.Status == StatusCode.Success ) {
          foreach ( var package in s_listPackagesRequest.Result ) {
            var data = s_dependencyData.FirstOrDefault( depData => depData.Name == package.name );
            if ( data == null )
              continue;
            data.UpdateStatus( DependencyData.State.Installed );
          }
        }
        else {
          Debug.LogError( s_listPackagesRequest.Error.message );
        }

        foreach ( var depData in s_dependencyData ) {
          if ( depData.Status == DependencyData.State.Unknown )
            depData.UpdateStatus( DependencyData.State.NotInstalled );
        }

        EditorApplication.update -= InitializeDependencies;
        s_listPackagesRequest = null;

        IsInitializing = false;
      }
    }

    /// <summary>
    /// Delete downloaded example package from the temporary directory.
    /// </summary>
    /// <param name="example">Example to delete the package for.</param>
    private static void DeleteTemporaryPackage( Example example )
    {
      try {
        var data = GetData( example );
        if ( data != null ) {
          data.DownloadedPackage?.Delete();
          data.DownloadedPackage = null;
        }
        else
          File.Delete( $"{TempDirectory.FullName}/{ExampleData.FindDirectoryName( example )}.unitypackage" );
      }
      catch ( Exception ) {
      }
    }

    private static ListRequest s_listPackagesRequest = null;
    private static List<AddRequest> s_addPackageRequests = new List<AddRequest>();
    private static ExampleData[] s_exampleData = new ExampleData[ (int)Example.NumExamples ];
    private static DependencyData[] s_dependencyData = new DependencyData[ (int)PackageDependency.NumDependencies ];
    private static Regex s_examplesPageRegex = new Regex( @"_downloads/(\w+)/AGXUnity_(\w+).unitypackage", RegexOptions.Compiled );
    private static DirectoryInfo s_tempDirectory = null;
    private static SerializedObject s_playerSettings = null;
    private static readonly object s_downloadCompleteLock = new object();
  }
}
