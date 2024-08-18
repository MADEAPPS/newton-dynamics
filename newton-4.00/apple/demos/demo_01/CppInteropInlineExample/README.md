#  CppInteropInlineExample

This project is for testing C++ and Swift in the same source base in the context of a an XCode project.

Require XCode 15, this example done on beta 6.
Apple Swift version 5.9 (swiftlang-5.9.0.128.2 clang-1500.0.40.1) 

## References

- https://developer.apple.com/videos/play/wwdc2023/10172/
- https://developer.apple.com/documentation/Swift/MixingLanguagesInAnXcodeProject 
- https://www.swift.org/documentation/cxx-interop/
- https://www.swift.org/documentation/cxx-interop/project-build-setup/
- https://developer.apple.com/documentation/Swift/MixingLanguagesInAnXcodeProject
- https://developer.apple.com/documentation/swift/callingapisacrosslanguageboundaries

## Related Repo
- https://github.com/carlynorama/CxxInteropLibrary/


## Getting Started

### Make the project

- Selected multi-target project
- Updated `Swift Compiler - Language` build settings for project to `C++/ObjectiveC++` (filter on "interop", its at the bottom) [Time Code 2:56](https://developer.apple.com/videos/play/wwdc2023/10172/)


### Add a C++ file

Choose the options to create header AND bridging header.

```
//
//  CxxExampleCode.hpp
//  CppInteropInlineExample
//
//  Created by Carlyn Maw on 8/9/23.
//

#ifndef CxxExampleCode_hpp
#define CxxExampleCode_hpp

#include <stdio.h>

int myFavoriteNumber();

#endif /* CxxExampleCode_hpp */

```


```
//
//  CxxExampleCode.cpp
//  CppInteropInlineExample
//
//  Created by Carlyn Maw on 8/9/23.
//

#include "CxxExampleCode.hpp"

int myFavoriteNumber() {
    return 5;
}
```

```
//
//  CppInteropInlineExample-Bridging-Header.h
//  CppInteropInlineExample
//
//  Created by Carlyn Maw on 8/9/23.
//
//
//  Use this file to import your target's public headers that you would like to expose to Swift.
//

#include "SimpleCxxFunctions.hpp"
```

### Add test call to ContentView.swift

```swift
struct ContentView: View {
    let number = myFavoriteNumber()
    var body: some View {
        VStack {
            Image(systemName: "globe")
                .imageScale(.large)
                .foregroundStyle(.tint)
            Text("Hello, No. \(number)!")
        }
        .padding()
    }
}
```
