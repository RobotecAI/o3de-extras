{
    "Source" : "ExampleEffect.azsl",
 
    "DepthStencilState" : { 
        "Depth" : { "Enable" : false }
    },

   "ProgramSettings": {
        "EntryPoints": [
        {
            "name" : "MainCS",
            "type" : "Compute"
        }
        ]
    },

    "DrawList": "forward"
}
