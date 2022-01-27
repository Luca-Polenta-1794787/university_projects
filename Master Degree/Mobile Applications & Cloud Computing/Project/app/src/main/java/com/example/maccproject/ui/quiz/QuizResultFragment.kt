package com.example.maccproject.ui.quiz

import android.content.Intent
import android.media.MediaPlayer
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.fragment.app.Fragment
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentQuizResultBinding
import androidx.navigation.findNavController
import kotlin.properties.Delegates
import android.text.method.ScrollingMovementMethod
import android.util.Log
import android.view.*
import android.widget.Toast
import androidx.activity.OnBackPressedCallback
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.database.FirebaseDatabase


class QuizResultFragment : Fragment() {

    data class Results(
        val title: String,
        val text: String,
        val description: String,
        val share: String)

    // The first answer is the correct one.  We randomize the answers before showing the text.
    // All questions must have four answers.  We'd want these to contain references to string
    // resources so we could internationalize. (or better yet, not define the questions in code...)
    private val results: MutableList<Results> = mutableListOf(
        Results(
            share = "a JEDI",
            title = "You are a JEDI",
            text = "Good is your way, defender of freedom and justice. Strength flows strong in you",
            description = "A Jedi is a member of the Jedi Order who studied, served, and used the mystical energies of the Force. Jedi fight to bring peace and justice to the Galactic Republic, usually against the Sith, their arch-enemies as well as devotees of the Dark Side, but also against the prophets of the Dark Side. In the Jedi Temple of Coruscant resides the Jedi Council, which is the highest authority in the Order. Within the Order there is a very strict division into Classes. The most powerful and experienced Jedi, also called Jedi Masters, seek out children between the ages of 4 and 7 in whom the Force, measured in Midichlorians, is very powerful, in order to make them in turn Jedi. The new apprentices receive the rank of Padawan and progress, followed by one or more Masters, up to the rank of Jedi Knight, and then independently up to the rank of Jedi Master. Jedi must also follow precise rules, such as controlling emotions, not killing unarmed opponents, and cannot have romantic relationships (though many Jedi practice them secretly) with each other or with others."
        ),
        Results(
            share = "poised in the force",
            title = "The force is poised with you",
            text = "It is not yet clear to which side of the force you belong. Your fate is uncertain",
            description = "Your situation still precarious. You have not yet chosen whether to be on the dark side of the force or not. Certainly the powerful, dark secrets of the Sith to dominate and conquer the universe appeal to you, but perhaps deep down you know that the balance, peace and republic promoted by the Jedi is what the universe most deserves. "
        ),
        Results(
            share = "a SITH",
            title = "You are a SITH",
            text = "The allure of power is something few can resist. You are a disciple of evil",
            description = "The Sith Order is a group that used the dark side without fear or limitation of any kind. Originally, the term \"Sith\" referred to the native species of the planet Korriban that was later subjugated and ruled by the Dark Jedi exiled by the Galactic Republic. Following centuries of cross-breeding (cultural and genetic), the word \"Sith\" was no longer used to identify a race, but rather membership and dedication to the philosophy and teachings of the dark side. Throughout their long history, the Sith commanded many empires and waged many wars and military campaigns; their influence gave rise to many cults and minor orders, which were technically not part of the Sith Order. Their members, however, combining their vision of the galaxy with ancient teachings, allowed the dark side to stand the test of time and once again cast its shadow over all living things. Such cults included the Naddists, the Disciples of Ragnos, the Krath, and the Sisters of the Night."
        ),
    )

    private var accumulatedPoints by Delegates.notNull<Int>()
    private var mediaPlayer: MediaPlayer? = null
    private val tagQuizResult:String = "QuizResultFragment"
    private var idxShare by Delegates.notNull<Int>()

    private var _binding: FragmentQuizResultBinding? = null
    private val binding get() = _binding!!

    private var lengthOfStop by Delegates.notNull<Int>()
    private var playing = false

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
                              savedInstanceState: Bundle?): View {
        _binding = FragmentQuizResultBinding.inflate(inflater, container, false)

        val root: View = binding.root

        //Hide back button
        if(activity is AppCompatActivity){
            (activity as AppCompatActivity?)?.supportActionBar?.setDisplayHomeAsUpEnabled(false)
        }
        // Read the points made to know which result to report
        accumulatedPoints = arguments?.getInt("accumulatedPoints")!!
        val resultQuiz: String
        when {
            accumulatedPoints>3 -> {
                idxShare = 0
                binding.imageResultQuiz.setImageResource(R.drawable.risultato_jedi2)
                binding.imageResultQuiz.setBackgroundResource(R.drawable.bordergreen)
                resultQuiz = "You are a Jedi knight!"
                mediaPlayer = MediaPlayer.create(context, R.raw.finale_buono)
            }
            accumulatedPoints==3 -> {
                idxShare = 1
                binding.imageResultQuiz.setImageResource(R.drawable.risultato_indeciso)
                binding.imageResultQuiz.setBackgroundResource(R.drawable.borderyellow)
                resultQuiz = "The force is still uncertain in you"
                mediaPlayer = MediaPlayer.create(context, R.raw.risultato_indeciso)
            }
            else -> {
                idxShare = 2
                binding.imageResultQuiz.setImageResource(R.drawable.risultato_sith)
                binding.imageResultQuiz.setBackgroundResource(R.drawable.borderred)
                resultQuiz = "You are a fearsome Sith"
                mediaPlayer = MediaPlayer.create(context, R.raw.marcia_imperiale)
            }
        }
        binding.titleQuizResult.text=results[idxShare].title
        binding.subtitleQuizResultBox.text=results[idxShare].text
        binding.infoResQuiz.text=results[idxShare].description

        binding.infoResQuiz.movementMethod = ScrollingMovementMethod()

        binding.tryAgainButton.setOnClickListener @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        { v: View ->
            v.findNavController().navigate(R.id.action_quiz_result_to_navigation_quiz)
        }
        //Save result on Firebase
        binding.saveResultButton.setOnClickListener @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        { view: View ->
            val mAuth = FirebaseAuth.getInstance()
            val currentUser = mAuth.currentUser
            val database = FirebaseDatabase.getInstance()
            Log.d(tagQuizResult, "User: " + currentUser?.uid.toString())
            database.reference.child("Users").child(currentUser?.uid.toString()).child("quiz").setValue(resultQuiz)
            Toast.makeText(activity?.applicationContext,"Result saved correctly" , Toast.LENGTH_SHORT).show()
        }

        activity?.onBackPressedDispatcher?.addCallback(viewLifecycleOwner, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                // in here you can do logic when backPress is clicked
                root.findNavController().navigate(R.id.action_quiz_result_to_navigation_quiz)
            }
        })

        binding.musicButton.setOnClickListener @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        { view: View ->
            if(playing){
                mediaPlayer!!.pause()
                lengthOfStop=mediaPlayer!!.currentPosition
                playing = false
                binding.musicButton.setImageResource(R.drawable.baseline_play_arrow_black_48)
                root.invalidate()
            }else{
                mediaPlayer!!.seekTo(lengthOfStop)
                mediaPlayer!!.start()
                playing = true
                binding.musicButton.setImageResource(R.drawable.baseline_pause_black_48)
                root.invalidate()
            }

        }

        // Imposto cosa succede quando finisce l'audio, cioè lo azzero per ricominciarlo se l'utente vuole
        mediaPlayer!!.setOnCompletionListener {
            lengthOfStop = 0
            playing = false
            binding.musicButton.setImageResource(R.drawable.baseline_play_arrow_black_48)
            root.invalidate()
        }

        setHasOptionsMenu(true)

        binding.musicButton.setImageResource(R.drawable.baseline_pause_black_48)
        mediaPlayer!!.start() // no need to call prepare(); create() does that for you
        playing = true
        return root
    }

    override fun onStop() {
        super.onStop()
        mediaPlayer?.release()
        mediaPlayer = null
    }

    override fun onCreateOptionsMenu(menu: Menu, inflater: MenuInflater) {
        super.onCreateOptionsMenu(menu, inflater)
        inflater.inflate(R.menu.quiz_share_menu, menu)
        // check if the activity resolves
        if (null == getShareIntent().resolveActivity(requireActivity().packageManager)) {
            // hide the menu item if it doesn't resolve
            menu.findItem(R.id.share)?.isVisible = false
        }
    }

    // Sharing from the Menu
    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        when (item.itemId) {
            R.id.share -> shareSuccess()
        }
        return super.onOptionsItemSelected(item)
    }

    // Creating our Share Intent
    private fun getShareIntent() : Intent {
        val shareIntent = Intent(Intent.ACTION_SEND)
        val messageToShare = "Guess what! I took the best star wars test and I discovered that I'm " +results[idxShare].share+
                "! Come do it yourself at the link \n" +
                "https://play.google.com/store/apps/details?id=com.maccproject.starwarsapp.page"
        shareIntent.setType("text/plain")
            .putExtra(Intent.EXTRA_TEXT, messageToShare)
        return shareIntent
    }

    // Starting an Activity with our new Intent
    private fun shareSuccess() {
        startActivity(getShareIntent())
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

}