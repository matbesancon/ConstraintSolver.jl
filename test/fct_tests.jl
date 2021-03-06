@testset "Function tests" begin

@testset "Logs"  begin

    var_states = Dict{Int64,Vector{Int64}}()
    var_changes = Dict{Int64,Vector{Tuple{Symbol, Int64, Int64, Int64}}}()
    children = CS.TreeLogNode[]
    l1 = CS.TreeLogNode(0,:Open,0,0,0,0,var_states, var_changes, children) 
    l2 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children) 
    @test !CS.same_logs(l1,l2) 

    # different children order
    children = CS.TreeLogNode[]
    tln1 = CS.TreeLogNode(0,:Open,0,1,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln2 = CS.TreeLogNode(0,:Open,0,2,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    push!(children, tln1)
    push!(children, tln2)
    l1 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children[[1,2]]) 
    l2 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children[[2,1]])
    @test !CS.same_logs(l1,l2) 

    # missing one child
    children = CS.TreeLogNode[]
    tln1 = CS.TreeLogNode(0,:Open,0,1,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln2 = CS.TreeLogNode(0,:Open,0,2,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    push!(children, tln1)
    push!(children, tln2)
    l1 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children[[1,2]]) 
    l2 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children[[1]])
    @test !CS.same_logs(l1,l2) 

    # 2 layer children different struture
    children1 = CS.TreeLogNode[]
    tln13 = CS.TreeLogNode(0,:Open,0,3,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln11 = CS.TreeLogNode(0,:Open,0,1,0,0,var_states, var_changes, [tln13]) 
    tln12 = CS.TreeLogNode(0,:Open,0,2,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    push!(children1, tln11)
    push!(children1, tln12)
    children2 = CS.TreeLogNode[]
    tln23 = CS.TreeLogNode(0,:Open,0,3,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln21 = CS.TreeLogNode(0,:Open,0,1,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln22 = CS.TreeLogNode(0,:Open,0,2,0,0,var_states, var_changes, [tln23]) 
    push!(children2, tln21)
    push!(children2, tln22)
    l1 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children1) 
    l2 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children2)
    @test !CS.same_logs(l1,l2)

    # 2 layer children different position
    children1 = CS.TreeLogNode[]
    tln13 = CS.TreeLogNode(0,:Open,0,3,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln14 = CS.TreeLogNode(0,:Open,0,4,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln11 = CS.TreeLogNode(0,:Open,0,1,0,0,var_states, var_changes, [tln13,tln14]) 
    tln12 = CS.TreeLogNode(0,:Open,0,2,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    push!(children1, tln11)
    push!(children1, tln12)
    children2 = CS.TreeLogNode[]
    tln23 = CS.TreeLogNode(0,:Open,0,3,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln24 = CS.TreeLogNode(0,:Open,0,4,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    tln21 = CS.TreeLogNode(0,:Open,0,1,0,0,var_states, var_changes, [tln24, tln23]) 
    tln22 = CS.TreeLogNode(0,:Open,0,2,0,0,var_states, var_changes, CS.TreeLogNode[]) 
    push!(children2, tln21)
    push!(children2, tln22)
    l1 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children1) 
    l2 = CS.TreeLogNode(0,:Closed,0,0,0,0,var_states, var_changes, children2)
    @test !CS.same_logs(l1,l2) 
end

end