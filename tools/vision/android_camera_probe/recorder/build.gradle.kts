plugins {
    id("com.android.application")
}

android {
    namespace = "com.nightfall.hfrrecorder"
    compileSdk = 36
    buildToolsVersion = "36.0.0"

    defaultConfig {
        applicationId = "com.nightfall.hfrrecorder"
        minSdk = 31
        targetSdk = 36
        versionCode = 17
        versionName = "0.5.3"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
}
