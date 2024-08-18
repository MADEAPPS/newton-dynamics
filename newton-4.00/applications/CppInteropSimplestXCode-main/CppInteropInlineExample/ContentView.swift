//
//  ContentView.swift
//  CppInteropInlineExample
//
//  Created by Labtanza on 8/9/23.
//

import SwiftUI

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

#Preview {
    ContentView()
}
