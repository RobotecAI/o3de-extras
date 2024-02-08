{
    "Source" : "ExampleEffect.azsl",
 
    "DepthStencilState" : { 
        "Depth" : { "Enable" : false }
    },

   "ProgramSettings": {
        "EntryPoints": [
            {
                "name": "MainVS",
                "type": "Vertex"
            }
        ]
    },

    "DrawList": "forward"
}
