# Miscellaneous utilities

using DataStructures: DataStructures, BinaryHeap

"""
    lazy_collect(collection)
    lazy_collect(element_type, collection)

Lazily `collect` an iterable. Returns the original `collection` if it is already
an `Array` of the appropriate type.
"""
lazy_collect(collection) = collect(collection)
lazy_collect(collection::Array) = collection
lazy_collect(element_type::Type, collection) = collect(element_type, collection)
lazy_collect(::Type{T}, collection::Array{T}) where {T} = collection

"Generate `Base.hash` for a user-defined (composite) type."
macro auto_hash(type)
    expr = quote
        fields = fieldnames($(esc(type)))
        eval(auto_hash_expr($(esc(type)), fields))
    end
    return expr
end

function auto_hash_expr(type, fields)
    function expand(i)
        if i == 0
            :(h)
        else
            :(hash(a.$(fields[i]), $(expand(i-1))))
        end
    end
    return quote
        function Base.hash(a::$(type), h::UInt)
            $(expand(length(fields)))
        end
    end
end

"Generate `Base.(==)` and `Base.isqual` for a user-defined (composite) type."
macro auto_equals(type)
    expr = quote
        names = fieldnames($(esc(type)))
        eval(auto_equals_expr($(esc(type)), names))
    end
    return expr
end

function auto_equals_expr(type, fields)
    function expand(i, f)
        if i == 0
            :true
        else
            :($f(a.$(fields[i]), b.$(fields[i])) && $(expand(i-1, f)))
        end
    end
    return quote
        function Base.:(==)(a::$(type), b::$(type))
            $(expand(length(fields), :(==)))
        end
        function Base.isequal(a::$(type), b::$(type))
            $(expand(length(fields), :isequal))
        end
    end
end

"Convert vector of unnormalized scores to probabiities."
function softmax(scores)
    T = eltype(scores)
    isempty(scores) && return Vector{T}()
    ws = exp.(scores .- maximum(scores))
    z = sum(ws)
    return isnan(z) ? ones(T, length(scores)) ./ length(scores) : ws ./ z
end

"Mix `probs` with an `epsilon` chance of a random element."
function add_epsilon_probs!(probs, epsilon)
    n = length(probs)
    probs .= (probs .* (1 - epsilon)) .+ (epsilon / n)
    return probs
end

"Return sample from the standard Gumbel distribution."
function randgumbel(rng::AbstractRNG=Random.GLOBAL_RNG)
    return -log(-log(rand(rng)))
end

unzip_pairs(ps) = unzip_pairs(collect(ps))
unzip_pairs(ps::AbstractDict) = collect(keys(ps)), collect(values(ps))
unzip_pairs(ps::AbstractArray{<:Pair}) = first.(ps), last.(ps)

"Compact formatting of types."
function compact_type_str(str::AbstractString, max_type_param_length::Int = 80)
    m = match(r"^(\w+)\{(.+)\}$", str)
    if !isnothing(m) && length(m.captures[2]) > max_type_param_length
        str = m.captures[1] * "{...}"
    end
    return str
end
compact_type_str(T::Type, max_type_param_length::Int = 80) =
    compact_type_str(repr(T), max_type_param_length)
compact_type_str(@nospecialize(x), max_type_param_length::Int = 80) =
    compact_type_str(typeof(T), max_type_param_length)

"Pretty printing for lists."
function print_list(io::IO, list::AbstractArray,
                    n_used::Int = 0, formatter = identity)
    indent = get(io, :indent, "")
    n_lines, _ = displaysize(io)
    n_lines -= n_used
    if length(list) > n_lines
        for x in @view(list[1:(n_lines÷2-1)])
            println(io)
            print(io, indent, "  ", formatter(x))
        end
        println(io)
        print(io, indent, "  ", "⋮")
        for x in @view(list[end-(n_lines÷2)+1:end])
            println(io)
            print(io, indent, "  ", formatter(x))
        end
    else
        for x in list
            println(io)
            print(io, indent, "  ", formatter(x))
        end
    end
    return nothing
end

"Pretty printing for data structures, adapted from `Base.dump`."
function show_struct(
    io::IO, @nospecialize(x);
    indent = "", show_fields = (), show_fields_compact = (),
    show_list = (), show_pddl_list = (), show_pddl = (),
    max_type_param_length::Int = (displaysize(io)[2]*3)÷4 -length(indent)
)
    T = typeof(x)
    T_str = compact_type_str(T, max_type_param_length)
    if isa(x, Function)
        print(io, x, " (function of type ", T_str, ")")
    else
        print(io, T_str)
    end
    nf = nfields(x)
    if nf > 0
        for field in 1:nf
            println(io)
            fname = fieldname(T, field)
            print(io, indent, "  ", string(fname), ": ")
            if !isdefined(x, field)
                print(io, Base.undef_ref_str)
                continue
            end
            val = getfield(x, field)
            if fname in show_fields
                if isa(val, NamedTuple)
                    show_struct(io, val; indent = indent * "  ")
                else
                    io_ctx = IOContext(io, :indent => indent * "  ")
                    show(io_ctx, MIME"text/plain"(), val)
                end
            elseif fname in show_fields_compact
                io_ctx = IOContext(io, :compact => true)
                show(io_ctx, val)
            elseif fname in show_list && isa(val, AbstractArray)
                summary(io, val)
                io_ctx = IOContext(io, :indent => indent * "  ")
                print_list(io_ctx, val, nf+1)
            elseif fname in show_pddl_list && isa(val, AbstractArray)
                summary(io, val)
                io_ctx = IOContext(io, :indent => indent * "  ") 
                print_list(io_ctx, val, nf+1, write_pddl)
            elseif fname in show_pddl && isa(val, Term)
                print(io, write_pddl(val))
            elseif isbits(val) || isa(val, Symbol) || isa(val, String)
                show(io, val)
            else
                new_max = max_type_param_length - 2
                print(io, compact_type_str(summary(val), new_max))
            end
        end
    elseif !isa(x, Function)
        print(io, " ")
        show(io, x)
    end
    nothing
end

"""
    FastPriorityQueue{K, V}([ord])

Fast heap-based priority queue implementation which supports enqueuing and
dequeueing, but not priority adjustment.
"""
struct FastPriorityQueue{K, V, O <: Base.Ordering}
    heap::BinaryHeap{Pair{K, V}, Base.Order.By{typeof(last), O}}
end 

function FastPriorityQueue{K, V}(
    ord::Base.Ordering = DataStructures.FasterForward()
) where {K, V}
    heap = BinaryHeap{Pair{K, V}}(Base.By(last, ord))
    return FastPriorityQueue{K, V, typeof(ord)}(heap)
end

function FastPriorityQueue{K, V}(
    ord::Base.Ordering, pair::Union{Pair{K, V}, Tuple{K, V}},
    pairs::Union{Pair{K, V}, Tuple{K, V}}...
) where {K, V}
    pq = FastPriorityQueue{K, V}(ord)
    enqueue!(pq, pair)
    for kv in pairs
        enqueue!(pq, kv)
    end
    return pq
end

function FastPriorityQueue{K, V}(ord::Base.Ordering, pairs) where {K, V}
    pairs = pairs isa AbstractVector ? pairs : collect(pairs)
    heap = BinaryHeap{Pair{K, V}}(Base.By(last, ord), pairs)
    return FastPriorityQueue{K, V, typeof(ord)}(heap)
end

FastPriorityQueue{K, V}(pairs) where {K, V} =
    FastPriorityQueue{K, V}(DataStructures.FasterForward(), pairs)

FastPriorityQueue(pairs::AbstractArray{Pair{K, V}}) where {K, V} =
    FastPriorityQueue{K, V}(DataStructures.FasterForward(), pairs)

Base.length(pq::FastPriorityQueue) = length(pq.heap)
Base.isempty(pq::FastPriorityQueue) = isempty(pq.heap)
Base.first(pq::FastPriorityQueue) = first(pq.heap)
Base.peek(pq::FastPriorityQueue) = peek(pq.heap)

Base.empty!(pq::FastPriorityQueue) =  (empty!(pq.heap.valtree); pq)

function Base.append!(pq::FastPriorityQueue, pairs)
    empty!(pq.heap.valtree)
    append!(pq.heap.valtree, pairs)
    DataStructures.heapify!(pq.heap.valtree, pq.heap.ordering)
    return pq
end

DataStructures.enqueue!(pq::FastPriorityQueue{K, V}, kv::Pair{K, V}) where {K, V} =
    (push!(pq.heap, kv); return pq)
DataStructures.enqueue!(pq::FastPriorityQueue{K, V}, k::K, v::V) where {K, V} =
    (push!(pq.heap, k => v); return pq)
DataStructures.dequeue!(pq::FastPriorityQueue) =
    first(pop!(pq.heap))
DataStructures.dequeue_pair!(pq::FastPriorityQueue) =
    pop!(pq.heap)
